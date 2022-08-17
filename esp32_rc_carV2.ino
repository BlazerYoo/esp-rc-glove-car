// Import required libraries

// ESP-NOW
#include <ESP8266WiFi.h>
#include <espnow.h>


// TB6612FNG motor driver
#include <SparkFun_TB6612.h>



/*---------------------TB6612FNG---------------------*/
// Motor
#define AIN1 D3
#define BIN1 D5
#define AIN2 D2
#define BIN2 D6
#define PWMA D1
#define PWMB D7
#define STBY D4

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = -1;
const int offsetB = 1;

// Motor initialization takes care of all pinModes
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void moveForward() {
  motor1.drive(230);
}

void moveBackward() {
  motor1.drive(-230);
}

void turnLeft() {
  motor2.drive(-230);
}

void turnRight() {
  motor2.drive(230);
}

void stopMove() {
  brake(motor1, motor2);
}


/*---------------------ESP-NOW---------------------*/
// Datatype of data to receive (must be same as data sent)
typedef struct test_struct {
  int x;
} test_struct;


// Initialize data
test_struct myData;


// Print and copy data received
//  Run this callback function when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(myData.x);

  if (myData.x == 1) moveForward();
  else if (myData.x == 2) moveBackward();
  else if (myData.x == 3) turnLeft();
  else if (myData.x == 4) turnRight();
  else if (myData.x == 5) stopMove();
}


// Setup
void setup() {

  // Initialize serial monitor
  Serial.begin(115200);
  
  // Set device as Wifi station
  WiFi.mode(WIFI_STA);

  //Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register self as receiver
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  // Register OnDataRecv() as callback function
  esp_now_register_recv_cb(OnDataRecv);
}


// Loop
void loop() {

}
