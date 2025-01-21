#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define BUFFER_SIZE 50                        // Buffer size
#define BNO055_SAMPLERATE_MS 100
#define SDA 21
#define SCL 22

int bufferIndex = 0;                          // Index for the buffer
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 500;    // Publish every 500 ms

// Buffer size and data structure
struct IMUData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
};

// WiFi
const char *ssid = "iPhone";              // Enter your WiFi name
const char *password = "ab12cd34ef56";    // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "172.20.10.3";
const char *topic = "imu_data";
const int mqtt_port = 1883;

Adafruit_BNO055 myIMU = Adafruit_BNO055();
IMUData imuBuffer[BUFFER_SIZE];           // Circular buffer for IMU data
WiFiClient espClient;
PubSubClient client(espClient);

void initializeSensor();
void initializeNetwork();
void callback(char *topic, byte *payload, unsigned int length);

void publishIMUData();
void clearBuffer();

void setup() {
  Serial.begin(115200);
  initializeSensor();
  initializeNetwork();
  Serial.println("System ready!");
}

void loop() {
  // Read IMU data
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Store IMU data in the buffer
  imuBuffer[bufferIndex].accX = acc.x();
  imuBuffer[bufferIndex].accY = acc.y();
  imuBuffer[bufferIndex].accZ = acc.z();
  imuBuffer[bufferIndex].gyroX = gyro.x();
  imuBuffer[bufferIndex].gyroY = gyro.y();
  imuBuffer[bufferIndex].gyroZ = gyro.z();
  imuBuffer[bufferIndex].magX = mag.x();
  imuBuffer[bufferIndex].magY = mag.y();
  imuBuffer[bufferIndex].magZ = mag.z();

  // Increment buffer index (circular buffer)
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Debug: Print accelerometer data
  // Serial.print("Accelerometer: ");
  // Serial.print(acc.x());
  // Serial.print(", ");
  // Serial.print(acc.y());
  // Serial.print(", ");
  // Serial.println(acc.z());

  // Check if it's time to publish the buffered data
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime >= publishInterval) {
    publishIMUData();                     // Publish buffered data
    Serial.println("Sending Buffer to MQTT broker...");
    lastPublishTime = currentTime;        // Update the last publish time
  }

  // Call the MQTT client loop
  client.loop();

  delay(10);  // 10 ms delay for 100 Hz sampling
}

void initializeSensor() {
  Wire.begin(SDA, SCL, 400000);
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void initializeNetwork() {
  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the Wi-Fi network");

  // Connecting to an MQTT broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (true) {  // Keep trying until the test message is sent successfully
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the MQTT broker\n", client_id.c_str());

    if (client.connect(client_id.c_str())) {
      Serial.println("Connected to the MQTT broker");

      // Send a test message
      const char* testTopic = "imu_data";
      const char* testPayload = "Test message from ESP32";
      if (client.publish(testTopic, testPayload)) {
        Serial.println("Test message published successfully. System is ready!");
        break;  // Exit the loop if the test message is sent successfully
      } else {
        Serial.println("Failed to publish test message. Retrying...");
        client.disconnect();  // Disconnect and retry
        delay(1000);
      }
    } else {
      Serial.print("Connection failed with state: ");
      Serial.println(client.state());
      delay(1000);  // Retry delay
    }
  }
}

void publishIMUData() {
  // Variables to store the sum of the last 10 samples
  float avgAccX = 0, avgAccY = 0, avgAccZ = 0;
  float avgGyroX = 0, avgGyroY = 0, avgGyroZ = 0;
  float avgMagX = 0, avgMagY = 0, avgMagZ = 0;

  // Sum the last 10 samples
  for (int i = 0; i < 10; i++) {
    int sampleIndex = (bufferIndex - 10 + i + BUFFER_SIZE) % BUFFER_SIZE;
    avgAccX += imuBuffer[sampleIndex].accX;
    avgAccY += imuBuffer[sampleIndex].accY;
    avgAccZ += imuBuffer[sampleIndex].accZ;
    avgGyroX += imuBuffer[sampleIndex].gyroX;
    avgGyroY += imuBuffer[sampleIndex].gyroY;
    avgGyroZ += imuBuffer[sampleIndex].gyroZ;
    avgMagX += imuBuffer[sampleIndex].magX;
    avgMagY += imuBuffer[sampleIndex].magY;
    avgMagZ += imuBuffer[sampleIndex].magZ;
  }

  // Calculate the average
  avgAccX /= 10;
  avgAccY /= 10;
  avgAccZ /= 10;
  avgGyroX /= 10;
  avgGyroY /= 10;
  avgGyroZ /= 10;
  avgMagX /= 10;
  avgMagY /= 10;
  avgMagZ /= 10;

  // Create a payload with the averaged IMU data
  char payload[128];  // Adjust the size based on your payload requirements
  int offset = 0;

  offset += snprintf(payload + offset, sizeof(payload) - offset,
                     "{\"AccX\":%.2f,\"AccY\":%.2f,\"AccZ\":%.2f,"
                     "\"GyroX\":%.2f,\"GyroY\":%.2f,\"GyroZ\":%.2f,"
                     "\"MagX\":%.2f,\"MagY\":%.2f,\"MagZ\":%.2f}",
                     avgAccX, avgAccY, avgAccZ,
                     avgGyroX, avgGyroY, avgGyroZ,
                     avgMagX, avgMagY, avgMagZ);

  // Debug: Print the payload
  Serial.println("Payload to publish:");
  Serial.println(payload);

  // Publish the JSON payload to the MQTT broker
  if (client.publish("imu_data", payload)) {
    Serial.println("IMU data published to MQTT broker.");
  } else {
    Serial.println("Failed to publish IMU data.");
    Serial.print("MQTT client state: ");
    Serial.println(client.state());  // Print the MQTT client state
  }
}

void clearBuffer() {
  // Reset the buffer by setting all values to 0
  for (int i = 0; i < BUFFER_SIZE; i++) {
    imuBuffer[i].accX = 0;
    imuBuffer[i].accY = 0;
    imuBuffer[i].accZ = 0;
    imuBuffer[i].gyroX = 0;
    imuBuffer[i].gyroY = 0;
    imuBuffer[i].gyroZ = 0;
    imuBuffer[i].magX = 0;
    imuBuffer[i].magY = 0;
    imuBuffer[i].magZ = 0;
  }
  bufferIndex = 0;  // Reset the buffer index
  Serial.println("Buffer cleared.");
}