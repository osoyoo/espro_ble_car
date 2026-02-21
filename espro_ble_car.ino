/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 * Arduino Smart Car Tutorial Lesson 1
 * Tutorial URL https://osoyoo.com/?p=11370
 * CopyRight www.osoyoo.com
 *
 * Modified to add BLE (Bluetooth Low Energy) control functionality
 *
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DEVICE_NAME     "ESP32_RobotCar"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

#define speedPinR 16    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  23    //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2  25    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 17    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  26    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2  27   //Left Motor direction pin 1 to MODEL-X IN4
#define TURN_SPEED 80
int SPEED=0;
// Command structure
struct Command {
  char action;       // M, B, L, R, X, Y, E
  int direction;     // -2, -1, 0, 1, 2
  int speed;         // 0-240
  int gear;          // 0 or 1
};
bool parseCommand(String commandStr, Command &cmd) {
  int commaIndex1 = commandStr.indexOf(',');
  int commaIndex2 = commandStr.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = commandStr.indexOf(',', commaIndex2 + 1);

  if (commaIndex1 == -1 || commaIndex2 == -1 || commaIndex3 == -1) {
    Serial.println("Error: Invalid command format");
    return false;
  }

  // Parse action field
  String actionStr = commandStr.substring(0, commaIndex1);
  actionStr.trim();
  if (actionStr.length() > 0) {
    cmd.action = actionStr.charAt(0);
  } else {
    return false;
  }

  // Parse direction field
  String directionStr = commandStr.substring(commaIndex1 + 1, commaIndex2);
  directionStr.trim();
  cmd.direction = directionStr.toInt();

  // Parse speed field
  String speedStr = commandStr.substring(commaIndex2 + 1, commaIndex3);
  speedStr.trim();
  cmd.speed = speedStr.toInt();

  // Parse gear field
  String gearStr = commandStr.substring(commaIndex3 + 1);
  gearStr.trim();
  cmd.gear = gearStr.toInt();

  // Validate speed range
  if (cmd.speed < 0) cmd.speed = 0;
  if (cmd.speed > 240) cmd.speed = 240;

  return true;
}
/*motor control*/
void go_Advance(int L_SPEED=0,int R_SPEED=0)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Left(int L_SPEED=TURN_SPEED,int R_SPEED=TURN_SPEED)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Right(int L_SPEED=TURN_SPEED,int R_SPEED=TURN_SPEED)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Back(int L_SPEED=0,int R_SPEED=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_BackLeft(int L_SPEED=TURN_SPEED,int R_SPEED=TURN_SPEED)  //Reverse and turn left
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_BackRight(int L_SPEED=TURN_SPEED,int R_SPEED=TURN_SPEED)  //Reverse and turn right
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

//Pins initialize
void init_GPIO()
{
	pinMode(RightMotorDirPin1, OUTPUT);
	pinMode(RightMotorDirPin2, OUTPUT);
	pinMode(speedPinL, OUTPUT);
	pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
}

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected");
      stop_Stop(); // Stop car when disconnected
      BLEDevice::startAdvertising(); // Restart advertising
    }
};

// BLE Characteristic Callbacks - Handle incoming commands
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      Command cmd;
      if (!parseCommand(value, cmd)) {
          Serial.println("Failed to parse command");
          return;
      }
      if (value.length() > 0) {
        char command = cmd.action;
        SPEED=cmd.speed;
        Serial.print("Received command: ");
        Serial.println(command);

        switch(command) {
          case 'M':  // Move forward
            go_Advance(SPEED,SPEED);
            break;
          case 'B':  // Move backward
            go_Back(SPEED,SPEED);
            break;
          case 'L':  // Turn left
            go_Left();
            break;
          case 'R':  // Turn right
            go_Right();
            break;
          case 'X':  // Back to left
            go_BackLeft();
            break;
          case 'Y':  // Back to right
            go_BackRight();
            break;
          case 'E':  // Stop
            stop_Stop();
            break;
          default:
            Serial.println("Unknown command");
            break;
        }
      }
    }
};

void setup()
{
  // Initialize Serial Monitor at 9600 baud
  Serial.begin(9600);
  Serial.println("ESP32 Robot Car BLE Control");
  Serial.println("Initializing...");

  // Initialize GPIO pins
  init_GPIO();
  stop_Stop(); // Ensure car starts in stopped state

  // Initialize BLE
  BLEDevice::init(BLE_DEVICE_NAME);

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // Print BLE information
  Serial.println("BLE Device is ready!");
  Serial.print("Device Name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("BLE Address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());
  Serial.println("Waiting for BLE connection...");
}

void loop(){
  // Main loop - BLE callbacks handle commands
  delay(100);
}
