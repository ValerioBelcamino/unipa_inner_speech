import json
import paho.mqtt.client as mqtt
import logging
import rclpy
from rclpy.node import Node
from common_msgs.msg import Intent

# Configurazione logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')

# Configurazione MQTT
# BROKER_HOST = "broker.emqx.io"  # Broker pubblico Eclipse Mosquitto per test
BROKER_HOST = "localhost" 
BROKER_PORT = 1883
TOPIC = "test/sensors/data"  # Topic di test (stesso del publisher)

class MQTTReceiver(Node):
    def __init__(self):
        super().__init__('mqtt_receiver_node')
        self.publisher = self.create_publisher(Intent, '/user_intent', 10)  # Create a publisher for /user_intent
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the MQTT broker
        self.client.connect(BROKER_HOST, BROKER_PORT, 60)
        self.client.loop_start()  # Start the MQTT loop
        
        self.angle_data = None
        self.heartrate_data = None

    def on_connect(self, client, userdata, flags, rc):
        """Callback chiamata quando il client si connette al broker"""
        if rc == 0:
            logging.info("Connesso al broker MQTT")
            client.subscribe(TOPIC)
            logging.info(f"Sottoscritto al topic: {TOPIC}")
        else:
            logging.error(f"Errore di connessione: {rc}")

    def on_message(self, client, userdata, msg):
        """Callback chiamata quando arriva un messaggio"""
        try:
            message_str = msg.payload.decode('utf-8')
            logging.info(f"Messaggio ricevuto sul topic {msg.topic}")

            publishing_trigger = False
            # Parsing JSON
            json_data = json.loads(message_str)
            if json_data["message_type"] == "heartrate_data":
                self.heartrate_data = json_data
            elif json_data["message_type"] == "angles_data":
                data_subset = {k: v for k, v in json_data.items() if k in ["direction", "side", "exercise", "nth_repetition"]}
                old_data_subset = {k: v for k, v in self.angle_data.items() if k in ["direction", "side", "exercise", "nth_repetition"]} if self.angle_data else None
                if data_subset != old_data_subset:
                    publishing_trigger = True
                self.angle_data = json_data

            # Process the JSON message and publish to ROS topic
            if publishing_trigger:
                self.process_and_publish()

        except json.JSONDecodeError as e:
            logging.error(f"Errore nel parsing JSON: {e}")
        except Exception as e:
            logging.error(f"Errore nel processare il messaggio: {e}")

    def process_and_publish(self):
        """Processa il messaggio JSON ricevuto e pubblica su ROS topic"""

        user_name = "Giulia"  # Example user name
        heart_rate = self.heartrate_data["_value"] if self.heartrate_data["_value"] else None
        angle = self.angle_data["angle"]
        nth_repetition = self.angle_data["nth_repetition"]
        exercise_name = self.angle_data["exercise"]
        side = self.angle_data["side"]
        direction = self.angle_data["direction"]
        
        # Create a new ROS message
        intent_msg = Intent()
        intent_msg.user_input = f"I'm {user_name}. \
            According to the sensor readings, am I performing the exercise correctly?\
            I'm doing the {nth_repetition} repetition of {exercise_name} exercise. \
            I'm moving my {side} limb {direction}, currently it is at {angle} degrees. \
            My current heart rate is {heart_rate} bpm."
        intent_msg.action_name = "SensorReadings"
        intent_msg.parameters = json.dumps({
            "heart_rate_readings": self.heartrate_data,
            "angles_readings": self.angle_data
        })

        # Publish the message to the /user_intent topic
        self.publisher.publish(intent_msg)
        logging.info(f"Pubblicato messaggio su /user_intent: {intent_msg}")

def main():
    rclpy.init()
    mqtt_receiver = MQTTReceiver()
    try:
        rclpy.spin(mqtt_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt_receiver.client.loop_stop()  # Stop the MQTT loop
        mqtt_receiver.client.disconnect()  # Disconnect from the MQTT broker
        rclpy.shutdown()

if __name__ == "__main__":
    main()