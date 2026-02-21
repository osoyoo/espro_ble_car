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
 * and MPU6050 gyroscope with PID control for precise movement
 * Using MPU6050_light and PID libraries
 *
 * BLE Command Format: Action,Direction,Speed,Gear
 * Example: M,0,60,1 (Move forward at speed 60)
 *          X,2,70,0 (Move backward facing 10 o'clock at speed 70)
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <PID_v1.h>

// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DEVICE_NAME     "ESP32_RobotCar"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Motor pins
#define speedPinR 16    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  23    //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2  25    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 17    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  26    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2  27   //Left Motor direction pin 1 to MODEL-X IN4

// MPU6050 object
MPU6050 mpu(Wire);

// PID variables for heading control
double currentHeading = 0.0;
double targetHeading = 0.0;
double pidOutput = 0.0;

// PID parameters (Kp, Ki, Kd)
double Kp = 2.0;      // Proportional gain
double Ki = 0.1;      // Integral gain
double Kd = 0.5;      // Derivative gain

// PID controller object
PID headingPID(&currentHeading, &pidOutput, &targetHeading, Kp, Ki, Kd, DIRECT);

// Movement state
enum MovementState {
  IDLE,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING_TO_DIRECTION
};
MovementState currentState = IDLE;

// Current movement parameters
int currentSpeed = 140;  // Current movement speed
float targetDirection = 0.0;  // Target direction angle

// Command structure
struct Command {
  char action;       // M, B, L, R, X, Y, E
  int direction;     // -2, -1, 0, 1, 2
  int speed;         // 0-240
  int gear;          // 0 or 1
};

/*
 * Normalize angle to [-180, 180] range
 */
float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

/*
 * Convert direction field to angle
 * Direction: -2, -1, 0, 1, 2
 * -2 = 2 o'clock = -60° (right)
 * -1 = 1 o'clock = -30° (right)
 *  0 = 12 o'clock = 0° (straight)
 *  1 = 11 o'clock = +30° (left)
 *  2 = 10 o'clock = +60° (left)
 */
float directionToAngle(int direction) {
  return direction * 30.0;
}

/*
 * Parse BLE command string: "Action,Direction,Speed,Gear"
 * Example: "M,0,60,1" or "X,2,70,0"
 */
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

/*
 * Set motor speeds with direction
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(LeftMotorDirPin1, HIGH);
    digitalWrite(LeftMotorDirPin2, LOW);
    analogWrite(speedPinL, abs(leftSpeed));
  } else if (leftSpeed < 0) {
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, HIGH);
    analogWrite(speedPinL, abs(leftSpeed));
  } else {
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, LOW);
    analogWrite(speedPinL, 0);
  }

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(RightMotorDirPin1, HIGH);
    digitalWrite(RightMotorDirPin2, LOW);
    analogWrite(speedPinR, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, HIGH);
    analogWrite(speedPinR, abs(rightSpeed));
  } else {
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, LOW);
    analogWrite(speedPinR, 0);
  }
}

/*
 * Move straight with PID heading control
 */
void moveStraightPID(bool forward) {
  // Update sensor data
  mpu.update();

  // Get current Z-axis angle (yaw)
  currentHeading = mpu.getAngleZ();

  // Compute PID correction
  headingPID.Compute();

  // Apply correction to motors
  int baseSpeed = forward ? currentSpeed : -currentSpeed;
  int leftSpeed = baseSpeed - (int)pidOutput;
  int rightSpeed = baseSpeed + (int)pidOutput;

  // Ensure speeds stay within valid range
  if (abs(leftSpeed) > 255) leftSpeed = (leftSpeed > 0) ? 255 : -255;
  if (abs(rightSpeed) > 255) rightSpeed = (rightSpeed > 0) ? 255 : -255;

  setMotors(leftSpeed, rightSpeed);
}

/*
 * Perform precise angle turn to target direction
 */
bool turnToDirection(float targetAngle) {
  // Update sensor data
  mpu.update();
  currentHeading = mpu.getAngleZ();

  float angleDifference = normalizeAngle(targetAngle - currentHeading);

  // Check if we've reached the target (within 2 degrees tolerance)
  if (abs(angleDifference) < 2.0) {
    stop_Stop();
    return true; // Turn complete
  }

  // Determine turn direction and speed based on angle difference
  int turnSpeed = currentSpeed * 0.7; // Use 70% of current speed for turning

  if (angleDifference > 0) {
    // Turn left (counterclockwise)
    setMotors(-turnSpeed, turnSpeed);
  } else {
    // Turn right (clockwise)
    setMotors(turnSpeed, -turnSpeed);
  }

  return false; // Turn in progress
}

/*motor control - basic functions*/
void go_Advance(int L_SPEED, int R_SPEED)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Left(int L_SPEED, int R_SPEED)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Right(int L_SPEED, int R_SPEED)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,L_SPEED);
  analogWrite(speedPinR,R_SPEED);
}
void go_Back(int L_SPEED, int R_SPEED)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
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
      currentState = IDLE;
      BLEDevice::startAdvertising(); // Restart advertising
    }
};

// BLE Characteristic Callbacks - Handle incoming commands
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();

      if (value.length() > 0) {
        Serial.print("Received: ");
        Serial.println(value);

        // Parse command
        Command cmd;
        if (!parseCommand(value, cmd)) {
          Serial.println("Failed to parse command");
          return;
        }

        // Print parsed values
        Serial.print("Action: ");
        Serial.print(cmd.action);
        Serial.print(", Direction: ");
        Serial.print(cmd.direction);
        Serial.print(", Speed: ");
        Serial.print(cmd.speed);
        Serial.print(", Gear: ");
        Serial.println(cmd.gear);

        // Update current speed
        currentSpeed = cmd.speed;

        // Process action
        switch(cmd.action) {
          case 'M':  // Move forward with heading control
            currentState = MOVING_FORWARD;
            mpu.update();
            targetHeading = mpu.getAngleZ(); // Lock current heading
            headingPID.SetMode(AUTOMATIC);
            Serial.print("Moving forward at speed ");
            Serial.print(currentSpeed);
            Serial.print(", locked heading: ");
            Serial.println(targetHeading);
            break;

          case 'B':  // Move backward with heading control
            currentState = MOVING_BACKWARD;
            mpu.update();
            targetHeading = mpu.getAngleZ(); // Lock current heading
            headingPID.SetMode(AUTOMATIC);
            Serial.print("Moving backward at speed ");
            Serial.print(currentSpeed);
            Serial.print(", locked heading: ");
            Serial.println(targetHeading);
            break;

          case 'L':  // Turn to direction (forward left)
          case 'R':  // Turn to direction (forward right)
            {
              float directionAngle = directionToAngle(cmd.direction);
              mpu.update();
              targetDirection = normalizeAngle(mpu.getAngleZ() + directionAngle);
              currentState = TURNING_TO_DIRECTION;
              Serial.print("Turning to direction: ");
              Serial.print(directionAngle);
              Serial.print("° (target heading: ");
              Serial.print(targetDirection);
              Serial.println(")");
            }
            break;

          case 'X':  // Turn to direction (backward left) - reverse angle
          case 'Y':  // Turn to direction (backward right) - reverse angle
            {
              float directionAngle = -directionToAngle(cmd.direction); // Negate for backward
              mpu.update();
              targetDirection = normalizeAngle(mpu.getAngleZ() + directionAngle);
              currentState = TURNING_TO_DIRECTION;
              Serial.print("Turning backward to direction: ");
              Serial.print(directionAngle);
              Serial.print("° (target heading: ");
              Serial.print(targetDirection);
              Serial.println(")");
            }
            break;

          case 'E':  // Stop
            stop_Stop();
            currentState = IDLE;
            headingPID.SetMode(MANUAL);
            Serial.println("Stopped");
            break;

          default:
            Serial.print("Unknown action: ");
            Serial.println(cmd.action);
            break;
        }
      }
    }
};

void setup()
{
  // Initialize Serial Monitor at 9600 baud
  Serial.begin(9600);
  Serial.println("ESP32 Robot Car BLE Control with MPU6050");
  Serial.println("Command Format: Action,Direction,Speed,Gear");
  Serial.println("Initializing...");

  // Initialize GPIO pins
  init_GPIO();
  stop_Stop(); // Ensure car starts in stopped state

  // Initialize I2C for MPU6050
  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("Failed to initialize MPU6050!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Calibrate gyroscope
  Serial.println("Calibrating gyroscope... Keep the car still!");
  delay(1000);
  mpu.calcOffsets(); // Calculate gyro and accelerometer offsets
  Serial.println("MPU6050 calibration complete!");

  // Configure PID controller
  headingPID.SetOutputLimits(-100, 100); // Limit correction to ±100
  headingPID.SetSampleTime(20); // 20ms sample time
  headingPID.SetMode(MANUAL); // Start in manual mode

  Serial.println("PID controller initialized");

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
  Serial.println("\nExample commands:");
  Serial.println("  M,0,60,1  - Move forward at speed 60");
  Serial.println("  B,0,80,0  - Move backward at speed 80");
  Serial.println("  L,1,100,1 - Turn to 11 o'clock (30° left)");
  Serial.println("  R,-1,100,1 - Turn to 1 o'clock (30° right)");
  Serial.println("  X,2,70,0  - Turn to 10 o'clock (60° left)");
  Serial.println("  E,0,0,0   - Stop");
}

void loop(){
  // Handle different movement states with PID control
  switch(currentState) {
    case MOVING_FORWARD:
      moveStraightPID(true); // Move forward with heading control
      break;

    case MOVING_BACKWARD:
      moveStraightPID(false); // Move backward with heading control
      break;

    case TURNING_TO_DIRECTION:
      if (turnToDirection(targetDirection)) {
        currentState = IDLE;
        Serial.println("Turn complete");
      }
      break;

    case IDLE:
    default:
      // Do nothing, waiting for commands
      delay(50);
      break;
  }

  delay(20); // Small delay for loop stability
}
