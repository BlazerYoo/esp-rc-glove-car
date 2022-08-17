// Import required libraries

// ICM20948
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ESP-NOW
#include <esp_now.h>
#include <WiFi.h>



/*---------------------ICM20948---------------------*/
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11



/*---------------------ESP-NOW---------------------*/
// ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x84, 0xF3, 0xEB, 0x2F, 0xD6, 0x0F};


// Datatype of data to send
typedef struct test_struct {
  int x;
} test_struct;


// Initialize data
test_struct test;


// Store peer information
esp_now_peer_info_t peerInfo;


// Print if message is successfully sent or not for which MAC address
//  Run this callback function when message is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



/*---------------------GLOVE HAND GESTURES---------------------*/
// Flex sensor pin def + vars
int indexPin = A2;
int middlePin = A3;
int ringPin = A4;
int pinkyPin = A9;
int thumbPin = A7;

int vibeMotorPin = A12;

int indexRead, middleRead, ringRead, pinkyRead, thumbRead;

bool indexBent = false;
bool middleBent = false;
bool ringBent = false;
bool pinkyBent = false;
bool thumbBent = false;


// Hand gesture threshold
int indexBentThres = 800;
int indexStr8Thres = 950;

int midBentThres = 1400;
int midStr8Thres = 1550;

int ringBentThres = 1400;
int ringStr8Thres = 1550;

int pinkyBentThres = 800;
int pinkyStr8Thres = 950;

int thumbBentThres = 800;
int thumbStr8Thres = 950;

int fingerReflex = 400;

int leftTurnThres = 3;
int rightTurnThres = -3;

// Forward
void moveFoward() {
  Serial.println("-----Move foward-----");
  digitalWrite(vibeMotorPin, HIGH);
  //delay(1);
  
  /*---------------------ESP-NOW---------------------*/
  // Assign value to each data attribute
  test.x = 1;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  //delay(5);
}


// Backward
void moveBackward() {
  Serial.println("-----Move backward-----");
  digitalWrite(vibeMotorPin, HIGH);
  //delay(10);
  
  /*---------------------ESP-NOW---------------------*/
  // Assign value to each data attribute
  test.x = 2;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  //delay(5);
}


// Left
void turnLeft() {
  Serial.println("-----Turn left-----");
  digitalWrite(vibeMotorPin, HIGH);
  //delay(10);
  
  /*---------------------ESP-NOW---------------------*/
  // Assign value to each data attribute
  test.x = 3;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  //delay(5);
}


// Right
void turnRight() {
  Serial.println("-----Turn right-----");
  digitalWrite(vibeMotorPin, HIGH);
  //delay(10);  
  
  /*---------------------ESP-NOW---------------------*/
  // Assign value to each data attribute
  test.x = 4;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  //delay(5);
}



// Stop
void stopMove() {
  Serial.println("-----Stop-----");
  digitalWrite(vibeMotorPin, LOW);
  
  /*---------------------ESP-NOW---------------------*/
  // Assign value to each data attribute
  test.x = 5;

  // Send message
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // Check message was sent successfully
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  //delay(5);
}




// Setup
void setup(void) {


  // Initialize serial monitor
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens


    
  /*---------------------ICM20948---------------------*/
  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();


  
  /*---------------------ESP-NOW---------------------*/
  // Set device as Wifi station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register OnDataSent() as callback function  
  esp_now_register_send_cb(OnDataSent);
   
  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


  
  /*---------------------GLOVE HAND GESTURES---------------------*/
  // Pin modes
  pinMode(indexPin, INPUT);
  pinMode(middlePin, INPUT);
  pinMode(ringPin, INPUT);
  pinMode(pinkyPin, INPUT);
  pinMode(thumbPin, INPUT);
  pinMode(vibeMotorPin, OUTPUT);

}



// Loop
void loop() {
  
  /*---------------------ICM20948---------------------*/
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);


  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.println(gyro.gyro.x);

  // Register direction gestures

  if (gyro.gyro.x >= leftTurnThres) turnLeft();
  else if (gyro.gyro.x <= rightTurnThres) turnRight();
  
  



  
  /*---------------------GLOVE HAND GESTURES---------------------*/
  indexBent = false;
  middleBent = false;
  ringBent = false;
  pinkyBent = false;
  thumbBent = false;

  // Read values
  indexRead = analogRead(indexPin);
  Serial.print("Index: ");
  Serial.println(indexRead);
  
  middleRead = analogRead(middlePin);
  Serial.print("Middle: ");
  Serial.println(middleRead);
  
  ringRead = analogRead(ringPin);
  Serial.print("Ring: ");
  Serial.println(ringRead);
  
  pinkyRead = analogRead(pinkyPin);
  Serial.print("Pinky: ");
  Serial.println(pinkyRead);

  thumbRead = analogRead(thumbPin);
  Serial.print("Thumb: ");
  Serial.println(thumbRead);

  // Register straight gestures
  //  Foward
  if (indexRead >= indexStr8Thres && thumbRead >= thumbStr8Thres
      && middleRead <= midBentThres) moveFoward();

  //  Backward
  else if (indexRead <= indexBentThres && thumbRead >= thumbStr8Thres
          && middleRead <= midBentThres) moveBackward();

  //  Stop
  else stopMove();
  
  Serial.println("________________________________________________________________");

  
  //delay(5);
}
