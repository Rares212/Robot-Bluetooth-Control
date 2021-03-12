#include <math.h>

#define DEBUG_MODE false

#define ADC_REF_VOLTAGE 5.0
#define MAX_PITCH_ANGLE 45.0
#define MAX_ROLL_ANGLE 30.0

float batteryVoltage = 8.0;
int batteryValue = 500;
int prevBatteryValue = 500;

const float turnVoltage = 2.5;
const float leftMotorVoltage = 6.0;
const float rightMotorVoltage = 5.8;
int maxLeftMotorSpeed = 200;
int maxRightMotorSpeed = 200;
int maxTurnSpeed = 90;

const float autonomousSpeedFactor = 0.4;

const int batteryVoltagePin = A0;
const float voltageDividerFactor = 2.0;

const int leftSensorPin = 8;
const int frontSensorPin = 7;
const int rightSensorPin = 9;

const int leftMotorPin1 = 4;
const int leftMotorPin2 = 5;
const int rightMotorPin1 = 3;
const int rightMotorPin2 = 2;
const int leftMotorSpeedPin = 11;
const int rightMotorSpeedPin = 10;

bool autonomous = false;
bool moveForward = false;
bool moveBackward = false;
bool moveLeft = false;
bool moveRight = false;

bool leftSensor = false;
bool rightSensor = false;
bool forwardSensor = false;

// Serial stuff
bool pitchRead = false;
bool parsingOrientationData = false;
char serialInput;
String pitchString;
String rollString;

//
bool gyroControl = false;
float pitch = 0;
float roll = 0;

unsigned long lastMeasurementTime = 0UL;
const unsigned long measurementTimeMs = 5000UL;

void driveMotors(bool leftDir, int leftSpeed, bool rightDir, int rightSpeed) {
  // Set left motor to drive forward
  if (leftDir) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  // Backward
  else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  // Same with the right motor
  if (rightDir) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  // Set motor speed
  analogWrite(leftMotorSpeedPin, leftSpeed);
  analogWrite(rightMotorSpeedPin, rightSpeed);
}

void sendMoveCommandsNormal(int forwardBackward, int leftRight, float speedFactor = 1.0) {
  // Just making sure
  //forwardBackward = constrain(forwardBackward, -1, 1);
  //leftRight = constrain(leftRight, -1, 1);
  int leftMotorSpeed = (int) maxLeftMotorSpeed * speedFactor;
  int rightMotorSpeed = (int) maxRightMotorSpeed * speedFactor;
  int turnSpeed = (int) maxTurnSpeed * speedFactor;

  // Forward
  if (forwardBackward == 1 && leftRight == 0)
    driveMotors(true, leftMotorSpeed, true, rightMotorSpeed);
  // Backward
  else if (forwardBackward == -1 && leftRight == 0)
    driveMotors(false, leftMotorSpeed, false, rightMotorSpeed);
  // Right
  else if (forwardBackward == 0 && leftRight == 1)
    driveMotors(true, turnSpeed, false, turnSpeed);
  // Left
  else if (forwardBackward == 0 && leftRight == -1)
    driveMotors(false, turnSpeed, true, turnSpeed);
  // Forward-Right
  else if (forwardBackward == 1 && leftRight == 1)
    driveMotors(true, leftMotorSpeed, true, turnSpeed);
  // Forward-Left
  else if (forwardBackward == 1 && leftRight == -1)
    driveMotors(true, turnSpeed, true, rightMotorSpeed);
  // Backward-Right
  else if (forwardBackward == -1 && leftRight == 1)
    driveMotors(false, turnSpeed, false, rightMotorSpeed);
  // Backward-Left
  else if (forwardBackward == -1 && leftRight == -1)
    driveMotors(false, leftMotorSpeed, false, turnSpeed);
  else
    driveMotors(true, 0, true, 0);
}

void sendMoveCommandsGyro(int leftMotorSpeed, int rightMotorSpeed, bool forward) {
  if (forward)
    driveMotors(true, leftMotorSpeed, true, rightMotorSpeed);
  else
    driveMotors(false, leftMotorSpeed, false, rightMotorSpeed);
}

void moveAutonomous() {
  int forwardBackward = 0;
  int leftRight = 0;
  // 000
  if (!leftSensor && !forwardSensor && !rightSensor) {
    forwardBackward = 1;
    leftRight = 0;
  }
  // 010
  else if (!leftSensor && forwardSensor && !rightSensor) {
    forwardBackward = -1;
    leftRight = 0;
  }
  // 100
  else if (leftSensor && !forwardSensor && !rightSensor) {
    forwardBackward = 1;
    leftRight = 1;
  }
  // 001
  else if (!leftSensor && !forwardSensor && rightSensor) {
    forwardBackward = 1;
    leftRight = -1;
  }
  // 110
  else if (leftSensor && forwardSensor && !rightSensor) {
    forwardBackward = 0;
    leftRight = 1;
  }
  // 011
  else if (!leftSensor && forwardSensor && rightSensor) {
    forwardBackward = 0;
    leftRight = -1;
  }
  // 101
  else if (leftSensor && !forwardSensor && rightSensor) {
    forwardBackward = -1;
    leftRight = 1;
  }
  // 111
  else if (leftSensor && forwardSensor && rightSensor) {
    forwardBackward = -1;
    leftRight = -1;
  }
  sendMoveCommandsNormal(forwardBackward, leftRight, autonomousSpeedFactor);
}

void moveButtons() {
  int forwardBackward = 0;
  int leftRight = 0;
  if (moveForward)
    forwardBackward += 1;
  if (moveBackward)
    forwardBackward -= 1;
  if (moveLeft)
    leftRight -= 1;
  if (moveRight)
    leftRight += 1;
  sendMoveCommandsNormal(forwardBackward, leftRight);
}

void moveGyro() {
  pitch = constrain(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
  roll = constrain(pitch, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
  float ySpeed = pitch / MAX_PITCH_ANGLE;
  float xSpeed = roll / MAX_ROLL_ANGLE;
  float angle = atan2(ySpeed, xSpeed) * RAD_TO_DEG;
  float magnitude = sqrt(ySpeed * xSpeed);
  int leftMotorSpeed, rightMotorSpeed;
  // True - forward, False - backward
  bool directionForward;
  // Forward-right turn
  if (angle >= 0.0 && angle < 90.0) {
    rightMotorSpeed = (int) maxRightMotorSpeed * angle / 90.0;
    leftMotorSpeed = maxLeftMotorSpeed;
    directionForward = true;
  }
  // Forward-left turn
  else if (angle >= 90.0 && angle <= 180.0) {
    rightMotorSpeed = maxRightMotorSpeed;
    leftMotorSpeed = (int) maxLeftMotorSpeed * (2.0 - angle / 90.0);
    directionForward = true;
  }
  // Backward-right turn
  else if (angle < 0.0 && angle > -90.0) {
    rightMotorSpeed = maxRightMotorSpeed;
    leftMotorSpeed = (int) maxLeftMotorSpeed * ((-angle) / 90.0);
    directionForward = false;
  }
  // Backward-left turn
  else if (angle <= -90.0 && angle >= -180) {
    rightMotorSpeed = (int) maxRightMotorSpeed * (2.0 - (-angle) / 90.0);
    leftMotorSpeed = (int) maxLeftMotorSpeed;
    directionForward = false;
  }
  else {
    rightMotorSpeed = 0;
    leftMotorSpeed = 0;
    directionForward = true;
  }
  sendMoveCommandsGyro(leftMotorSpeed, rightMotorSpeed, directionForward);
}

void moveRobot() {
  if (autonomous) {
    moveAutonomous();
  }
  else {
    if (!gyroControl)
      moveButtons();
    else
      moveGyro();
  }
}

/*
  float readVoltage(int pin, int nSamples = 32) {
  int sum = 0;
  for (int i = 0; i < nSamples; i++)
    sum += analogRead(pin);
  return (sum / nSamples) / 1023.0 * ADC_REF_VOLTAGE;
  }
*/

int analogReadMultiple(int pin, int nSamples = 10L) {
  long sum = 0;
  for (int i = 0; i < nSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(5);
  }
  return (int) sum / nSamples;
}

int batteryVoltageToPercentage(float voltage) {
  float percentage;
  if (voltage < 7.37)
    percentage = (int) (1.0753628256687998e+001 * voltage - 7.0777233782128093e+001);
  else
    percentage = (int) (-5.8281191355116931e+001 * voltage * voltage + 1.0039335953939719e+003 * voltage - 4.2232666419085963e+003);
  percentage = constrain(percentage, 0, 100);
  return percentage;
}

void setMaxMotorSpeeds() {
  maxLeftMotorSpeed = constrain((int) ((leftMotorVoltage / batteryVoltage) * 255), 0, 255);
  maxRightMotorSpeed = constrain((int) ((rightMotorVoltage / batteryVoltage) * 255), 0, 255);
  maxTurnSpeed = constrain((int) ((turnVoltage / batteryVoltage) * 255), 0, 255);
}

void setup() {
  Serial.begin(9600);
  pinMode(leftSensorPin, INPUT);
  pinMode(frontSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
}

void loop() {
  // Serial parsing
  if (Serial.available() > 0) {
    serialInput = Serial.read();
    if (parsingOrientationData) {
      if (!pitchRead) {
        if (isDigit(serialInput) || serialInput == '-' || serialInput == '.')
          pitchString += serialInput;
        else if (serialInput == ',')
          pitchRead = true;
      }
      else {
        if (isDigit(serialInput) || serialInput == '-' || serialInput == '.')
          rollString += serialInput;
        else if (serialInput == '\0' || serialInput == '\n') {
          pitchRead = false;
          parsingOrientationData = false;
          pitch = pitchString.toFloat();
          roll = rollString.toFloat();
        }
      }
    }
    else {
      if (serialInput == 'O')
        parsingOrientationData = true;
      else if (serialInput == 'F')
        moveForward = true;
      else if (serialInput == 'B')
        moveBackward = true;
      else if (serialInput == 'L')
        moveLeft = true;
      else if (serialInput == 'R')
        moveRight = true;
      /*
        else if (serialInput == 'N') {
        moveForward = false;
        moveBackward = false;
        moveLeft = false;
        moveRight = false;
        }
      */
      else if (serialInput == '1') {
        moveForward = false;
      }
      else if (serialInput == '2') {
        moveBackward = false;
      }
      else if (serialInput == '3') {
        moveLeft = false;
      }
      else if (serialInput == '4') {
        moveRight = false;
      }
      else if (serialInput == 'A')
        autonomous = true;
      else if (serialInput == 'M')
        autonomous = false;
      else if (serialInput == 'G')
        gyroControl = true;
      else if (serialInput == 'K')
        gyroControl = false;
    }
  }
  // Moving average for battery voltage
  prevBatteryValue = batteryValue;
  batteryValue = analogReadMultiple(batteryVoltagePin);
  batteryVoltage = (batteryValue + prevBatteryValue) / 2046.0 * ADC_REF_VOLTAGE * voltageDividerFactor;

  leftSensor = !digitalRead(leftSensorPin);
  forwardSensor = !digitalRead(frontSensorPin);
  rightSensor = !digitalRead(rightSensorPin);
  if (!DEBUG_MODE)
    moveRobot();
  else {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(leftMotorSpeedPin, 200);
    analogWrite(rightMotorSpeedPin, 200);
  }

  unsigned long currentTime = millis();
  if (currentTime > lastMeasurementTime + measurementTimeMs) {
    setMaxMotorSpeeds();
    Serial.print(batteryVoltageToPercentage(batteryVoltage));
    Serial.println("%");
    lastMeasurementTime = currentTime;
  }

}
