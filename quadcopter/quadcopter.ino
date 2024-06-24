#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

Adafruit_MPU6050 mpu;

// Define offsets for accelerometer and gyroscope
float accelOffsetX = 1.43;
float accelOffsetY = -0.2;
float accelOffsetZ = 10;
float gyroOffsetX = -0.06;
float gyroOffsetY = 0.02;
float gyroOffsetZ = -0.01;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int valueLeftX;
  int valueLeftY;
  int valueRightX;
  int valueRightY;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Left X: ");
  Serial.println(myData.valueLeftX);
  Serial.print("Left Y: ");
  Serial.println(myData.valueLeftY);
  Serial.print("Right X: ");
  Serial.println(myData.valueRightX);
  Serial.print("Right Y: ");
  Serial.println(myData.valueRightY);
  Serial.println();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);

  ledcAttachPin(D2, 0);
  ledCAttachPin(D3, 1);
  ledCAttachPin(D5, 2);
  ledCAttachPin(D6, 3);

  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("");
  delay(100);
}

void loop() {
  
  ledcWrite(0, 1000);
  ledcWrite(1, 1000);
  ledcWrite(2, 1000);
  ledcWrite(3, 1000);

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Apply offsets to the raw sensor data */
  float accelX = a.acceleration.x - accelOffsetX;
  float accelY = a.acceleration.y - accelOffsetY;
  float accelZ = a.acceleration.z - accelOffsetZ;
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;

  /* Print out the values with offsets applied */
  Serial.print("Acceleration X: ");
  Serial.print(accelX);
  Serial.print(", Y: ");
  Serial.print(accelY);
  Serial.print(", Z: ");
  Serial.print(accelZ);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(gyroX);
  Serial.print(", Y: ");
  Serial.print(gyroY);
  Serial.print(", Z: ");
  Serial.print(gyroZ);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}