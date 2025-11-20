import cv2
import requests
import json
import time
import uuid
import os
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO  # YOLOv8-seg para detecção de produtos por segmentação

# ======================= CONFIGURATION =======================
# ESP-CAM Configuration
ESP_CAM_IP = "192.168.5.148"  # Replace with your ESP-CAM IP
CAPTURE_URL = f"http://{ESP_CAM_IP}/capture"
STREAM_URL = f"http://{ESP_CAM_IP}/stream"

# MQTT Configuration
MQTT_BROKER_HOST = "35.198.11.30"
MQTT_BROKER_PORT = 1883
MQTT_USERNAME = "rpi-user"
MQTT_PASSWORD = "eRtL)5$H01L!"
MQTT_TOPIC_BASE = "store/loja1/shelf/gondolaA/events"

# AI Configuration - YOLOv8-seg pré-treinado
# Utiliza segmentação para detectar produtos específicos nas prateleiras.
# Recomenda-se treinar um modelo customizado (ex.: best-products-seg.pt) para classes próprias.
MODEL_PATH = "yolov8n-seg.pt"  # Modelo YOLOv8-seg pré-treinado
CONFIDENCE_THRESHOLD = 0.5  # Threshold para detecção de produtos
# =============================================================

# Load AI model
print("🔄 Carregando modelo YOLOv8-seg...")
model = YOLO(MODEL_PATH)
print("✅ Modelo carregado com sucesso!")

# Product database (map class IDs to product info)
PRODUCT_DATABASE = {
    0: {"name": "Coca-Cola 350ml", "sku": "SKU-COCA-350", "price": 7.50},
    1: {"name": "Pepsi 350ml", "sku": "SKU-PEPSI-350", "price": 6.99},
    2: {"name": "Guaraná 350ml", "sku": "SKU-GUARANA-350", "price": 6.49},
    3: {"name": "Água Mineral 500ml", "sku": "SKU-AGUA-500", "price": 4.90},
    # Adicione/atualize seus produtos conforme o treinamento do modelo
}

class ESPCamProcessor:
    def __init__(self):
        self.mqtt_client = self.setup_mqtt()
        self.detection_active = False
        self.last_detection_time = {}  # Inicializar tracking de interações
        
    def setup_mqtt(self):
        client = mqtt.Client()
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.on_connect = self.on_connect
        client.on_publish = self.on_publish
        client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
        client.loop_start()
        return client
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Conectado ao Broker MQTT")
        else:
            print(f"❌ Falha na conexão MQTT: {rc}")
            
    def on_publish(self, client, userdata, mid):
        print(f"📤 Evento publicado (mid={mid})")
    
    def capture_image(self):
        """Capture image from ESP-CAM"""
        try:
            response = requests.get(CAPTURE_URL, timeout=10)
            if response.status_code == 200:
                # Save image temporarily
                temp_path = "temp_capture.jpg"
                with open(temp_path, "wb") as f:
                    f.write(response.content)
                return temp_path
        except Exception as e:
            print(f"❌ Erro ao capturar imagem: {e}")
        return None
    
    def process_with_ai(self, image_path):
        """Process image with YOLOv8-seg para detecção de produtos"""
        try:
            results = model(image_path, conf=CONFIDENCE_THRESHOLD)
            
            detections = []
            for result in results:
                boxes = result.boxes
                masks = result.masks
                
                if boxes is not None:
                    for i, box in enumerate(boxes):
                        cls_id = int(box.cls.item())
                        confidence = box.conf.item()
                        
                        product_info = PRODUCT_DATABASE.get(
                            cls_id,
                            {"name": f"Produto_{cls_id}", "sku": f"SKU-{cls_id}", "price": 0.0},
                        )
                        bbox = box.xywh.tolist()[0]  # [x, y, width, height]
                        
                        segmentation = None
                        mask_area = None
                        mask_ratio = None
                        
                        if masks is not None and masks.data is not None and len(masks.data) > i:
                            mask_tensor = masks.data[i]
                            mask_array = mask_tensor.cpu().numpy()
                            mask_area = int(np.count_nonzero(mask_array))
                            total_pixels = mask_array.shape[0] * mask_array.shape[1]
                            mask_ratio = float(mask_area) / float(total_pixels) if total_pixels else 0.0
                            
                            try:
                                segmentation = masks.xy[i].tolist()
                            except Exception:
                                segmentation = None
                        
                        detection = {
                            "class_id": cls_id,
                            "product_name": product_info["name"],
                            "sku": product_info.get("sku"),
                            "price": float(product_info.get("price", 0.0)),
                            "confidence": confidence,
                            "bbox": bbox,
                            "mask_area": mask_area,
                            "mask_ratio": mask_ratio,
                            "segmentation": segmentation,
                        }
                        detections.append(detection)
            
            return detections
        except Exception as e:
            print(f"❌ Erro no processamento AI: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def send_event(self, detection, duration=0):
        """Send detection event via MQTT"""
        event_data = {
            "event_id": str(uuid.uuid4()),
            "device_id": "esp-cam-01",
            "shelf_id": "gondolaA",
            "zone_id": int(detection["bbox"][0] // 100),  # Simple zone calculation
            "product_name": detection["product_name"],
            "sku": detection.get("sku"),
            "price": detection.get("price"),
            "duration": duration,
            "confidence": float(detection["confidence"]),
            "timestamp_end": time.time()
        }
        
        if detection.get("mask_area") is not None:
            event_data["mask_area"] = detection["mask_area"]
        if detection.get("mask_ratio") is not None:
            event_data["mask_ratio"] = detection["mask_ratio"]
        if detection.get("segmentation") is not None:
            # Segmentation pode ser um payload grande; serialize apenas se necessário
            event_data["segmentation"] = detection["segmentation"]
        
        topic = f"{MQTT_TOPIC_BASE}/{event_data['device_id']}"
        self.mqtt_client.publish(topic, json.dumps(event_data))
        print(f"✅ Produto detectado: {event_data['product_name']} (Conf: {event_data['confidence']:.2f})")
    
    def track_interactions(self, detections):
        """Track how long produtos estão sendo manipulados"""
        current_time = time.time()
        
        for detection in detections:
            product_key = detection.get("sku") or detection["product_name"]
            
            # Verificar se é uma continuação da mesma interação
            if product_key in self.last_detection_time:
                time_diff = current_time - self.last_detection_time[product_key]
                if time_diff < 2.0:  # Mesma interação se dentro de 2 segundos
                    if time_diff >= 0.5:  # Enviar a cada 0.5s para interações contínuas
                        detection["duration"] = time_diff
                        self.send_event(detection, time_diff)
            else:
                # Novo produto detectado, enviar imediatamente
                detection["duration"] = 0
                self.send_event(detection, 0)
            
            # Atualizar último tempo de detecção
            self.last_detection_time[product_key] = current_time
    
    def run(self):
        """Main processing loop"""
        print("🚀 Iniciando processamento da ESP-CAM para detecção de produtos (segmentação)...")
        
        try:
            while True:
                # Capture image
                image_path = self.capture_image()
                if not image_path:
                    time.sleep(1)
                    continue
                
                # Process with AI
                detections = self.process_with_ai(image_path)
                
                if detections:
                    print(f"🛒 {len(detections)} produto(s) detectado(s)")
                    self.track_interactions(detections)
                else:
                    print("⏳ Nenhum produto detectado...")
                
                # Limpar arquivo temporário
                try:
                    if os.path.exists(image_path):
                        os.remove(image_path)
                except Exception as e:
                    print(f"⚠️ Erro ao remover arquivo temporário: {e}")
                
                # Adjust processing rate
                time.sleep(0.5)  # Process ~2 frames per second
                
        except KeyboardInterrupt:
            print("🛑 Encerrando processamento...")
        finally:
            # Limpar arquivos temporários
            try:
                if os.path.exists("temp_capture.jpg"):
                    os.remove("temp_capture.jpg")
            except:
                pass
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == "__main__":
    processor = ESPCamProcessor()
    processor.run()
