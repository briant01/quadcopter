/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

//A1-3 = remote 1
//D2-4 = remote 2

#define VRXLEFT_PIN  36 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRYLEFT_PIN  39 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
#define VRXRIGHT_PIN 34 // ESP32 pin GPIO34 (ADC0) connected to VRX pin
#define VRYRIGHT_PIN 35 // ESP32 pin GPIO35 (ADC0) connected to VRY pin

/*
#define VRXLEFT_PIN  D5 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRYLEFT_PIN  D3 // ESP32 pin GPIO39 (ADC0) connected to VRY pin

#define VRXRIGHT_PIN  A2 //pin
#define VRYRIGHT_PIN  A1 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
*/

int valueLeftX = 0; // to store the X-axis value
int valueLeftY = 0; // to store the Y-axis value

int valueRightX = 0; // to store the X-axis value
int valueRightY = 0; // to store the Y-axis value

//Mac Address: 08:D1:F9:22:BF:A0
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x22, 0xBF, 0xA0};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(9600);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  strcpy(myData.a, "THIS IS A CHAR");
  myData.b = random(1,20);
  myData.c = 1.2;
  myData.d = false;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);

  // read X and Y analog values
  valueLeftX = analogRead(VRXLEFT_PIN);
  valueLeftY = analogRead(VRYLEFT_PIN);
  valueRightX = analogRead(VRXRIGHT_PIN);
  valueRightY = analogRead(VRYRIGHT_PIN);

  // print data to Serial Monitor on Arduino IDE
  Serial.print("Left x = ");
  Serial.print(valueLeftX);
  Serial.print(", Left y = ");
  Serial.println(valueLeftY);

  Serial.print("Right x = ");
  Serial.print(valueRightX);
  Serial.print(", Right y = ");
  Serial.println(valueRightY);

  delay(200);
}