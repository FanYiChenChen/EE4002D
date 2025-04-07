#!/usr/bin/env python3

from picamera2 import Picamera2
import numpy as np
from PIL import Image
import time
from datetime import datetime
import json
import os
import paho.mqtt.client as mqtt
import logging
import base64
import threading
import queue
import joblib
from pathlib import Path
from io import BytesIO
import torch
import torchvision.transforms as transforms
import torchvision.models as models
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))


# Configuration
MQTT_BROKER = "mqtt.thingsboard.cloud"
MQTT_PORT = 1883
ACCESS_TOKEN = "5js5cq4re26yxvi0ytzy"
CLIENT_ID = "RPI"
MODEL_PATH = 'mlp_model.pkl'
NPK_MODEL_PATH = "lettuce_npk_resnet18_state_dict.pth"
ML_IMAGE_SIZE = 64
SAVE_DIR = Path.home() / 'plant_monitor_data'

# Create save directory if it doesn't exist
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(SAVE_DIR / 'plant_monitor.log'),
        logging.StreamHandler()
    ]
)

class PlantMonitor:
    def __init__(self):
        logging.info("Initializing Plant Monitor...")
        self._init_camera()
        self._load_models()
        
        # Create directory for storing images
        self.image_dir = 'plant_images'
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        
        # Initialize MQTT client
        self.client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
        self.client.username_pw_set(ACCESS_TOKEN)
        self.client.max_packet_size = 5242880  # 5MB
        
        # Set up MQTT callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish
        
        # Message queue and locks
        self.msg_queue = queue.Queue()
        self.camera_lock = threading.Lock()
        self.connected = False
        
        # Connect to MQTT
        self._connect_mqtt()
        
        # Start message processing thread
        self.process_thread = threading.Thread(target=self._process_messages)
        self.process_thread.daemon = True
        self.process_thread.start()

    def _init_camera(self):
        """Initialize camera with RGB format"""
        retry_count = 0
        while retry_count < 3:
            try:
                self.camera = Picamera2()
                preview_config = self.camera.create_preview_configuration(
                main={"size": (1280, 720), "format": "BGR888"},
                buffer_count=2
)
                self.camera.configure(preview_config)
                self.camera.start()
                time.sleep(2)  # Warm up time
                logging.info("Camera initialized successfully")
                break
            except Exception as e:
                retry_count += 1
                logging.error(f"Camera initialization attempt {retry_count} failed: {e}")
                if retry_count == 3:
                    raise Exception("Failed to initialize camera after 3 attempts")
                time.sleep(5)

    def _load_models(self):
        """Load ML models"""
        try:
            # Load the original health model
            self.model = joblib.load(MODEL_PATH)
            logging.info("ML health model loaded successfully")
            
            # Load NPK detection model (from test.py)
            self._init_npk_model()
            logging.info("NPK detection model loaded successfully")
        except Exception as e:
            logging.error(f"Failed to load models: {e}")
            raise
    
    def _init_npk_model(self):
        """Initialize and load the NPK detection model from test.py"""
        try:
            # Limit threads to avoid Raspberry Pi crashes
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
            
            # Set device to CPU
            self.device = torch.device("cpu")
            
            # Create ResNet18 model
            self.npk_model = self._get_resnet18()
            self.npk_model.load_state_dict(torch.load(NPK_MODEL_PATH, map_location=self.device))
            self.npk_model.to(self.device)
            self.npk_model.eval()
            
            # Class names for NPK detection
            self.npk_class_names = ["Full Nutrition", "Nitrogen Deficiency", "Phosphorus Deficiency", "Potassium Deficiency"]
            
            # Image transform for NPK model
            self.npk_transform = transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
            ])
        except Exception as e:
            logging.error(f"Failed to initialize NPK model: {e}")
            raise
    
    def _get_resnet18(self):
        """Create ResNet18 model structure with modified last layer"""
        model = models.resnet18(weights=None)
        num_features = model.fc.in_features
        model.fc = torch.nn.Linear(num_features, 4)  # 4 classes
        return model

    def _connect_mqtt(self):
        try:
            logging.info(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}")
            self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            logging.error(f"Failed to connect to MQTT broker: {e}")

    def preprocess_image_for_ml(self, rgb_array):
        """Preprocess image for ML prediction"""
        try:
            # Convert to PIL Image (already in RGB format)
            pil_image = Image.fromarray(rgb_array)
            # Convert to grayscale
            img_gray = pil_image.convert('L')
            # Resize
            img_resized = img_gray.resize((ML_IMAGE_SIZE, ML_IMAGE_SIZE))
            # Normalize and flatten
            img_normalized = np.array(img_resized).flatten() / 255.0
            # Expand dimensions for model
            img_data = np.expand_dims(img_normalized, axis=0)
            return img_data
        except Exception as e:
            logging.error(f"Error preprocessing image for ML: {e}")
            return None

    def predict_plant_health(self, image_data):
        """Predict plant health using ML model"""
        try:
            prediction = self.model.predict(image_data)
            health_status = 'Healthy' if prediction[0] == 0 else 'Unhealthy'
            return health_status
        except Exception as e:
            logging.error(f"Error predicting plant health: {e}")
            return None
    
    def predict_npk_status(self, pil_image):
        """Predict NPK status using the ResNet18 model"""
        try:
            # Convert image to tensor
            img_tensor = self.npk_transform(pil_image).unsqueeze(0).to(self.device)
            
            with torch.no_grad():
                output = self.npk_model(img_tensor)
                probabilities = torch.nn.functional.softmax(output, dim=1)
                class_idx = torch.argmax(probabilities, dim=1).item()
                confidence = probabilities[0, class_idx].item()
            
            return self.npk_class_names[class_idx], confidence
        except Exception as e:
            logging.error(f"Error predicting NPK status: {e}")
            return None, 0.0

    def capture_and_encode_image(self):
        """Capture and encode image using PIL"""
        with self.camera_lock:
            try:
                # Capture image in RGB format
                rgb_array = self.camera.capture_array()
                
                # Convert to PIL Image
                pil_image = Image.fromarray(rgb_array)
                
                # Save original image
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                image_path = f'{self.image_dir}/plant_{timestamp}.jpg'
                pil_image.save(image_path, 'JPEG', quality=90)
                
                # Process for ML health detection
                ml_image_data = self.preprocess_image_for_ml(rgb_array)
                health_status = self.predict_plant_health(ml_image_data) if ml_image_data is not None else None
                
                # Process for NPK detection
                npk_status, npk_confidence = self.predict_npk_status(pil_image)
                
                # Resize for web display
                target_height = 360
                width, height = pil_image.size
                scale = target_height / height
                new_size = (int(width * scale), target_height)
                resized_image = pil_image.resize(new_size, Image.Resampling.LANCZOS)
                
                # JPEG encoding
                buffer = BytesIO()
                resized_image.save(buffer, format='JPEG', quality=90, optimize=True)
                encoded_image = base64.b64encode(buffer.getvalue()).decode('utf-8')
                
                logging.info(f"Image captured and encoded. Original saved to: {image_path}")
                logging.info(f"NPK Status: {npk_status} (Confidence: {npk_confidence:.2f})")
                
                return encoded_image, health_status, npk_status, npk_confidence
                
            except Exception as e:
                logging.error(f"Error capturing/encoding image: {e}")
                return None, None, None, 0.0

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            logging.info("Successfully connected to MQTT broker")
            self.client.subscribe('v1/devices/me/rpc/request/+')
            self.publish_telemetry({'status': 'connected'})
        else:
            self.connected = False
            logging.error(f"Failed to connect with code: {rc}")

    def on_disconnect(self, client, userdata, rc):
        self.connected = False
        logging.warning(f"Disconnected from MQTT broker with code: {rc}")
        if rc != 0:
            time.sleep(5)
            self._connect_mqtt()

    def on_publish(self, client, userdata, mid):
        logging.debug(f"Message {mid} published successfully")

    def on_message(self, client, userdata, msg):
        try:
            request_id = msg.topic.split('/')[-1]
            payload = json.loads(msg.payload)
            method = payload.get('method', '')
            
            if method == 'captureImage':
                threading.Thread(target=self._handle_capture_request, 
                               args=(request_id,)).start()
        except Exception as e:
            logging.error(f"Error processing message: {e}")

    def _handle_capture_request(self, request_id):
        try:
            encoded_image, health_status, npk_status, npk_confidence = self.capture_and_encode_image()
            if encoded_image:
                # Send health status, NPK status and timestamp first
                telemetry_data = {
                    'plantHealth': health_status,
                    'npkStatus': npk_status,
                    'npkConfidence': f"{npk_confidence:.2f}",
                    'lastUpdated': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                self.msg_queue.put(('v1/devices/me/telemetry', 
                                  json.dumps(telemetry_data), 1))
                
                # Then send image separately
                image_data = {
                    'img': encoded_image
                }
                self.msg_queue.put(('v1/devices/me/telemetry', 
                                  json.dumps(image_data), 1))
                
                response = {
                    'success': True, 
                    'timestamp': int(time.time() * 1000),
                    'plantHealth': health_status,
                    'npkStatus': npk_status,
                    'npkConfidence': f"{npk_confidence:.2f}"
                }
                response_topic = f'v1/devices/me/rpc/response/{request_id}'
                self.msg_queue.put((response_topic, json.dumps(response), 1))
            else:
                response = {'success': False, 'error': 'Failed to capture image'}
                response_topic = f'v1/devices/me/rpc/response/{request_id}'
                self.msg_queue.put((response_topic, json.dumps(response), 1))
        except Exception as e:
            logging.error(f"Error handling capture request: {e}")

    def _process_messages(self):
        while True:
            try:
                msg = self.msg_queue.get()
                if msg is None:
                    break
                
                topic, payload, qos = msg
                if not self.connected:
                    time.sleep(1)
                    self.msg_queue.put(msg)
                    continue
                
                result = self.client.publish(topic, payload, qos)
                result.wait_for_publish()
                
                self.msg_queue.task_done()
                time.sleep(0.1)
                
            except Exception as e:
                logging.error(f"Error processing message: {e}")
                time.sleep(1)

    def publish_telemetry(self, data):
        try:
            if 'ts' not in data:
                data['ts'] = int(time.time() * 1000)
            self.msg_queue.put(('v1/devices/me/telemetry', json.dumps(data), 1))
        except Exception as e:
            logging.error(f"Error queueing telemetry: {e}")

    def monitor_plant(self, interval_minutes=1):
        logging.info(f"Starting plant monitoring with {interval_minutes} minute intervals")
        while True:
            try:
                if self.connected:
                    encoded_image, health_status, npk_status, npk_confidence = self.capture_and_encode_image()
                    if encoded_image:
                        # Send health status, NPK status and timestamp first
                        telemetry_data = {
                            'plantHealth': health_status,
                            'npkStatus': npk_status,
                            'npkConfidence': f"{npk_confidence:.2f}",
                            'lastUpdated': time.strftime('%Y-%m-%d %H:%M:%S')
                        }
                        self.publish_telemetry(telemetry_data)
                        
                        # Then send image separately
                        image_data = {
                            'img': encoded_image
                        }
                        self.publish_telemetry(image_data)
                        
                time.sleep(interval_minutes * 60)
            except Exception as e:
                logging.error(f"Error in monitoring loop: {e}")
                time.sleep(60)

    def cleanup(self):
        self.msg_queue.put(None)
        self.client.loop_stop()
        self.client.disconnect()
        self.camera.stop()
        logging.info("Cleanup completed")

if __name__ == "__main__":
    monitor = PlantMonitor()
    
    try:
        monitor.monitor_plant(interval_minutes=1)
    except KeyboardInterrupt:
        logging.info("\nMonitoring stopped by user")
    finally:
        monitor.cleanup()
