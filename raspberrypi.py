import cv2
import requests
import json
import time
import uuid
import os
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO  # YOLOv8-pose para detecção de mãos

# ======================= CONFIGURATION =======================
# ESP-CAM Configuration
ESP_CAM_IP = "192.168.0.79"  # Replace with your ESP-CAM IP
CAPTURE_URL = f"http://{ESP_CAM_IP}/capture"
STREAM_URL = f"http://{ESP_CAM_IP}/stream"

# MQTT Configuration
MQTT_BROKER_HOST = "35.198.11.30"
MQTT_BROKER_PORT = 1883
MQTT_USERNAME = "rpi-user"
MQTT_PASSWORD = "eRtL)5$H01L!"
MQTT_TOPIC_BASE = "events"

# AI Configuration - YOLOv8-pose pré-treinado
# NOTA: O YOLOv8-pose padrão detecta pessoas com 17 keypoints corporais (não mãos diretamente)
# A detecção de gestos é uma aproximação baseada na posição dos pulsos.
# Para detecção precisa de gestos de mãos, considere:
# - MediaPipe Hands (mais preciso, 21 keypoints por mão)
# - Modelo YOLOv8 treinado com dataset Ultralytics Hand Keypoints
MODEL_PATH = "yolov8n-pose.pt"  # Modelo YOLOv8-pose pré-treinado
CONFIDENCE_THRESHOLD = 0.5  # Threshold ajustado para detecção
# =============================================================

# Load AI model
print("🔄 Carregando modelo YOLOv8-pose...")
model = YOLO(MODEL_PATH)
print("✅ Modelo carregado com sucesso!")

# Gesture database (mapeia gestos detectados)
GESTURE_DATABASE = {
    "punho": {"name": "Punho Fechado", "action": "fechado"},
    "um": {"name": "Um Dedo", "action": "apontar"},
    "dois": {"name": "Dois Dedos", "action": "vitoria"},
    "tres": {"name": "Três Dedos", "action": "contagem"},
    "quatro": {"name": "Quatro Dedos", "action": "contagem"},
    "cinco": {"name": "Mão Aberta", "action": "aberto"},
    "ok": {"name": "OK", "action": "confirmacao"},
    "desconhecido": {"name": "Gesto Desconhecido", "action": "indefinido"}
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
    
    def detect_gesture_from_body_pose(self, body_keypoints):
        """Detecta gesto simplificado baseado na pose do corpo (YOLOv8-pose padrão tem 17 keypoints)"""
        # YOLOv8-pose padrão detecta pessoas com 17 keypoints:
        # 0: nariz, 1: olho esquerdo, 2: olho direito, 3: orelha esquerda, 4: orelha direita
        # 5: ombro esquerdo, 6: ombro direito, 7: cotovelo esquerdo, 8: cotovelo direito
        # 9: pulso esquerdo, 10: pulso direito, 11: quadril esquerdo, 12: quadril direito
        # 13: joelho esquerdo, 14: joelho direito, 15: tornozelo esquerdo, 16: tornozelo direito
        
        if body_keypoints is None or len(body_keypoints) < 17:
            return "desconhecido"
        
        # Usar posição dos pulsos para detectar gestos básicos
        left_wrist = body_keypoints[9] if body_keypoints[9][2] > 0.5 else None  # [x, y, visibility]
        right_wrist = body_keypoints[10] if body_keypoints[10][2] > 0.5 else None
        
        # Detecção simplificada baseada na posição dos pulsos
        # Esta é uma aproximação - para detecção precisa de gestos, use MediaPipe ou modelo customizado
        if left_wrist is not None and right_wrist is not None:
            # Calcular distância entre pulsos
            distance = np.sqrt((left_wrist[0] - right_wrist[0])**2 + (left_wrist[1] - right_wrist[1])**2)
            
            # Altura relativa dos pulsos
            avg_wrist_y = (left_wrist[1] + right_wrist[1]) / 2
            
            # Gestos simplificados baseados na posição
            if distance < 50:  # Mãos próximas
                return "punho"  # ou "mãos juntas"
            elif left_wrist[1] < body_keypoints[5][1] or right_wrist[1] < body_keypoints[6][1]:
                # Pulsos acima dos ombros - mão levantada
                return "cinco"  # mão aberta/levantada
            else:
                return "desconhecido"
        elif left_wrist is not None or right_wrist is not None:
            # Apenas uma mão visível
            wrist = left_wrist if left_wrist is not None else right_wrist
            shoulder = body_keypoints[5] if left_wrist is not None else body_keypoints[6]
            
            if wrist[1] < shoulder[1]:  # Pulso acima do ombro
                return "um"  # mão levantada
            else:
                return "desconhecido"
        
        return "desconhecido"
    
    def process_with_ai(self, image_path):
        """Process image with YOLOv8-pose para detecção de mãos e gestos"""
        try:
            results = model(image_path, conf=CONFIDENCE_THRESHOLD)
            
            detections = []
            for result in results:
                boxes = result.boxes
                keypoints = result.keypoints
                
                if boxes is not None and keypoints is not None:
                    for i, box in enumerate(boxes):
                        # Filtrar apenas detecções de pessoas (classe 0 no COCO)
                        cls_id = int(box.cls.item())
                        if cls_id != 0:  # Pular se não for pessoa
                            continue
                        
                        confidence = box.conf.item()
                        
                        # Obter keypoints do corpo (YOLOv8-pose padrão detecta pessoas com 17 keypoints)
                        # NOTA: Para detecção precisa de gestos de mãos, considere usar:
                        # - MediaPipe Hands (mais preciso para mãos)
                        # - Modelo YOLOv8 treinado com dataset Hand Keypoints
                        body_keypoints = None
                        
                        if keypoints.data is not None and len(keypoints.data) > i:
                            body_keypoints = keypoints.data[i].cpu().numpy()
                        
                        # Detectar gesto baseado na pose do corpo (aproximação)
                        gesture = "desconhecido"
                        if body_keypoints is not None and len(body_keypoints) >= 17:
                            gesture = self.detect_gesture_from_body_pose(body_keypoints)
                        
                        gesture_info = GESTURE_DATABASE.get(gesture, GESTURE_DATABASE["desconhecido"])
                        
                        detection = {
                            "class_id": cls_id,
                            "gesture": gesture,
                            "gesture_name": gesture_info["name"],
                            "gesture_action": gesture_info["action"],
                            "confidence": confidence,
                            "bbox": box.xywh.tolist()[0],  # [x, y, width, height]
                            "keypoints": body_keypoints.tolist() if body_keypoints is not None else None
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
            "gesture": detection["gesture"],
            "gesture_name": detection["gesture_name"],
            "gesture_action": detection["gesture_action"],
            "duration": duration,
            "confidence": float(detection["confidence"]),
            "timestamp_end": time.time()
        }
        
        # Adicionar keypoints se disponíveis (opcional, pode ser grande)
        if detection.get("keypoints") is not None:
            event_data["keypoints"] = detection["keypoints"]
        
        topic = f"{MQTT_TOPIC_BASE}/{event_data['device_id']}"
        self.mqtt_client.publish(topic, json.dumps(event_data))
        print(f"✅ Gesto detectado: {event_data['gesture_name']} (Conf: {event_data['confidence']:.2f})")
    
    def track_interactions(self, detections):
        """Track how long gestures are being performed"""
        current_time = time.time()
        
        for detection in detections:
            gesture_key = detection["gesture"]
            
            # Verificar se é uma continuação da mesma interação
            if gesture_key in self.last_detection_time:
                time_diff = current_time - self.last_detection_time[gesture_key]
                if time_diff < 2.0:  # Mesma interação se dentro de 2 segundos
                    # Só enviar evento se o gesto mudou ou após intervalo maior
                    if time_diff >= 0.5:  # Enviar a cada 0.5s para gestos contínuos
                        detection["duration"] = time_diff
                        self.send_event(detection, time_diff)
            else:
                # Novo gesto detectado, enviar imediatamente
                detection["duration"] = 0
                self.send_event(detection, 0)
            
            # Atualizar último tempo de detecção
            self.last_detection_time[gesture_key] = current_time
    
    def run(self):
        """Main processing loop"""
        print("🚀 Iniciando processamento da ESP-CAM para detecção de mãos e gestos...")
        
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
                    print(f"✋ {len(detections)} mão(s)/gesto(s) detectado(s)")
                    self.track_interactions(detections)
                else:
                    print("⏳ Nenhuma mão detectada...")
                
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
