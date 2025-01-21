import json
from confluent_kafka import Producer
import paho.mqtt.client as mqtt

# Kafka configuration
KAFKA_BROKER = "localhost:9092"     # Replace with your Kafka broker address
KAFKA_TOPIC = "imu"            # Replace with your Kafka topic

# MQTT configuration
MQTT_BROKER = "172.20.10.3"         # Replace with your MQTT broker address
MQTT_PORT = 1883                    # Default MQTT port
MQTT_TOPIC = "imu_data"             # Replace with your MQTT topic

# Kafka producer configuration
conf = {
    "bootstrap.servers": KAFKA_BROKER,
}

# Create a Kafka producer
producer = Producer(conf)

# Callback when a message is received from MQTT
def on_message(client, userdata, msg):
    try:
        # Decode the MQTT message payload
        payload = msg.payload.decode("utf-8")
        print(f"Received MQTT message: {payload}")

        # Forward the message to Kafka
        producer.produce(KAFKA_TOPIC, value=payload)
        producer.flush()  # Ensure the message is sent
        print(f"Forwarded to Kafka topic: {KAFKA_TOPIC}")
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

# MQTT client setup
client = mqtt.Client()
client.on_message = on_message

# Connect to the MQTT broker
client.connect(MQTT_BROKER, MQTT_PORT, 60)
print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")

# Subscribe to the MQTT topic
client.subscribe(MQTT_TOPIC)
print(f"Subscribed to MQTT topic: {MQTT_TOPIC}")

# Start the MQTT loop to listen for messages
print("Starting MQTT to Kafka bridge...")
client.loop_forever()