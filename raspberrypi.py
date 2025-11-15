import cv2
import requests
import json
import time
import uuid
import paho.mqtt.client as mqtt
from ultralytics import YOLO  # Your AI model

# ======================= CONFIGURATION =======================
# ESP-CAM Configuration
ESP_CAM_IP = "192.168.5.148"  # Replace with your ESP-CAM IP
CAPTURE_URL = f"http://{ESP_CAM_IP }/capture"
STREAM_URL = f"http://{ESP_CAM_IP }/stream"

# MQTT Configuration
MQTT_BROKER_HOST = "35.198.11.30"
MQTT_BROKER_PORT = 1883
MQTT_USERNAME = "rpi-user"
MQTT_PASSWORD = "eRtL)5$H01L!"
MQTT_TOPIC_BASE = "store/loja1/shelf/gondolaA/events"

# AI Configuration
MODEL_PATH = "best.pt"  # Your trained YOLO model
CONFIDENCE_THRESHOLD = 0.7
# =============================================================

# Load AI model
model = YOLO(MODEL_PATH)

# Product database (map class IDs to product info)
PRODUCT_DATABASE = {
    0: {"name": "Coca-Cola 350ml", "price": 75.0},
    1: {"name": "Pepsi 350ml", "price": 70.0},
    2: {"name": "Guarana 350ml", "price": 65.0},
    # Add your products here
}

class ESPCamProcessor:
    def __init__(self):
        self.mqtt_client = self.setup_mqtt()
        self.detection_active = False
        
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
                with open("temp_capture.jpg", "wb") as f:
                    f.write(response.content)
                return "temp_capture.jpg"
        except Exception as e:
            print(f"❌ Erro ao capturar imagem: {e}")
        return None
    
    def process_with_ai(self, image_path):
        """Process image with YOLO model"""
        try:
            results = model(image_path, conf=CONFIDENCE_THRESHOLD)
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls_id = int(box.cls.item())
                        confidence = box.conf.item()
                        product_info = PRODUCT_DATABASE.get(cls_id, {"name": f"Unknown_{cls_id}", "price": 0.0})
                        
                        detection = {
                            "class_id": cls_id,
                            "product_name": product_info["name"],
                            "confidence": confidence,
                            "price": product_info["price"],
                            "bbox": box.xywh.tolist()[0]  # [x, y, width, height]
                        }
                        detections.append(detection)
            
            return detections
        except Exception as e:
            print(f"❌ Erro no processamento AI: {e}")
            return []
    
    def send_event(self, detection, duration=0):
        """Send detection event via MQTT"""
        event_data = {
            "event_id": str(uuid.uuid4()),
            "device_id": "esp-cam-01",
            "shelf_id": "gondolaA",
            "zone_id": int(detection["bbox"][0] // 100),  # Simple zone calculation
            "product_name": detection["product_name"],
            "duration": duration,
            "confidence": float(detection["confidence"]),
            "price": float(detection["price"]),
            "timestamp_end": time.time()
        }
        
        topic = f"{MQTT_TOPIC_BASE}/{event_data['device_id']}"
        self.mqtt_client.publish(topic, json.dumps(event_data))
        print(f"✅ Evento detectado: {event_data['product_name']} (Conf: {event_data['confidence']:.2f})")
    
    def track_interactions(self, detections):
        """Track how long products are being interacted with"""
        current_time = time.time()
        
        for detection in detections:
            product_key = detection["product_name"]
            
            # Check if this is a continuing interaction
            if hasattr(self, 'last_detection_time') and product_key in self.last_detection_time:
                time_diff = current_time - self.last_detection_time[product_key]
                if time_diff < 2.0:  # Same interaction if within 2 seconds
                    detection["duration"] = time_diff
                    self.send_event(detection, time_diff)
            
            # Update last detection time
            if not hasattr(self, 'last_detection_time'):
                self.last_detection_time = {}
            self.last_detection_time[product_key] = current_time
    
    def run(self):
        """Main processing loop"""
        print("🚀 Iniciando processamento da ESP-CAM...")
        
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
                    print(f"📦 {len(detections)} produto(s) detectado(s)")
                    self.track_interactions(detections)
                else:
                    print("⏳ Nenhum produto detectado...")
                
                # Adjust processing rate
                time.sleep(0.5)  # Process ~2 frames per second
                
        except KeyboardInterrupt:
            print("🛑 Encerrando processamento...")
        finally:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == "__main__":
    processor = ESPCamProcessor()
    processor.run()
