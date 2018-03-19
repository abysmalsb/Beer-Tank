/**************************************************************
   This code controls a Beer Tank using an
   Arduino MKR1000 through serial or RC. More info:
   https://www.hackster.io/Abysmal/beer-tank-20a2ed

   This project is made for "3D Imaging with Walabot & Amazon Alexa"
   contest on hackster.io. More info:
   https://www.hackster.io/contests/walabot2017

   author: Balázs Simon
 **************************************************************/

#include <Servo.h>

/** Connection settings **/
#define LED_PIN A0
#define BATTERY_PIN A1
#define CRANE_FEEDBACK_PIN A2
#define LED_DOOR_PIN A3
#define RIGHT_MOTOR_B_PIN A5
#define RIGHT_MOTOR_A_PIN A6
#define STEPPER_MOTOR_COIL_2_PIN 0
#define STEPPER_MOTOR_COIL_1_PIN 1
#define RIGHT_MOTOR_SPEED_PIN 2
#define LEFT_HATCH_SERVO_PIN 3
#define RIGHT_HATCH_SERVO_PIN 4
#define LEFT_MOTOR_SPEED_PIN 5
#define RC_HORIZONTAL_PIN 6
#define RC_VERTICAL_PIN 7
#define RC_STATE_SWITCH_PIN 8
#define STEPPER_MOTOR_COIL_4_PIN 9
#define STEPPER_MOTOR_COIL_3_PIN 10
#define LEFT_MOTOR_B_PIN 11
#define LEFT_MOTOR_A_PIN 12

/** Software settings **/
#define RIGHT_SERVO_ZERO_ANGLE 170
#define LEFT_SERVO_ZERO_ANGLE 26
#define HATCH_OPENED_ANGLE 86
#define CRANE_ELEVATED_VALUE 245
#define CRANE_LOWERED_VALUE 115
#define LED_DOOR_CLOSED 45
#define LED_DOOR_OPEN 85
#define MIN_SPEED 20
#define RC_ZERO_TIME_FROM 1465
#define RC_ZERO_TIME_TO 1525
#define RC_MAX_TIME 2000
#define RC_MIN_TIME 950

/** Hardware settings **/
#define BAT_VD_R1 22000 // BAT = Battery, VD = Voltage Divider
#define BAT_VD_R2 10000
#define BAT_VD_VREF 3.3
#define ADC_MAX_VALUE 1023
#define MAX_PWM_RESOLUTION 255

/** Derived constants **/
// This equation is the combination of the voltage divider and ADC value to voltage converter equations
#define ADC_VALUE_TO_BAT_VOLTAGE_MULTIPLIER (BAT_VD_R1 + BAT_VD_R2) * BAT_VD_VREF / ADC_MAX_VALUE / BAT_VD_R2

volatile unsigned long horizontal_timer_start = 0;
volatile int horizontal_pulse_time;

volatile unsigned long vertical_timer_start = 0;
volatile int vertical_pulse_time;

volatile unsigned long state_switch_timer_start = 0;
volatile int state_switch_pulse_time;

volatile int motorSpeedBase = 0;
volatile int motorSpeedChange = 0;

Servo leftHatch;
Servo rightHatch;
Servo ledDoor;

int motorState = 8; // the motor is switched off
int elevated = 1;

boolean radioControlled = true;
boolean movingEnabled = true;
boolean lightingEnabled = false;

/**
 * Interrupt handling function for the RC joystick's horizontal part
 * In case of radio control, the PWM signal will be used to calculate the speed of the tank
 */
void calcHorizontal()
{
  if (digitalRead(RC_HORIZONTAL_PIN) == HIGH)
  {
    horizontal_timer_start = micros();
  }
  else
  {
    if (horizontal_timer_start != 0)
    {
      horizontal_pulse_time = ((volatile int)micros() - horizontal_timer_start);
      //restart the timer
      horizontal_timer_start = 0;
      if (timeValid(horizontal_pulse_time)) {
        motorSpeedChange = (horizontal_pulse_time - 1500) / 3;
      }
      else {
        motorSpeedChange = 0;
      }
    }
  }
}

/**
 * Interrupt handling function for the RC joystick's vertical part
 * In case of radio control, the PWM signal will be used to steer the tank
 */
void calcVertical()
{
  if (digitalRead(RC_VERTICAL_PIN) == HIGH)
  {
    vertical_timer_start = micros();
  }
  else
  {
    if (vertical_timer_start != 0)
    {
      vertical_pulse_time = ((volatile int)micros() - vertical_timer_start);
      //restart the timer
      vertical_timer_start = 0;
      if (timeValid(vertical_pulse_time)) {
        motorSpeedBase = (vertical_pulse_time - 1500) / 2;
      }
      else {
        motorSpeedBase = 0;
      }
    }
  }
}

/**
 * Interrupt handling function for the RC switch that will control the crane, the lighting and
 * in case of autonomous control it is used to safely disable the tank if it starts to do crazy things
 */
void calcSwitch()
{
  if (digitalRead(RC_STATE_SWITCH_PIN) == HIGH)
  {
    state_switch_timer_start = micros();
  }
  else
  {
    if (state_switch_timer_start != 0)
    {
      state_switch_pulse_time = ((volatile int)micros() - state_switch_timer_start);
      //restart the timer
      state_switch_timer_start = 0;
      if (radioControlled) {
        if (!elevated && state_switch_pulse_time < 1500) {
          lightingEnabled = !lightingEnabled;
          enableLighting(lightingEnabled);
        }
        elevated = state_switch_pulse_time < 1500;
      }
      else {
        movingEnabled = state_switch_pulse_time < 1500;
      }
    }
  }
}

/**
 * validating pulse width times
 */
boolean timeValid(int RCtime) {
  return (RCtime > RC_MIN_TIME && RCtime < RC_ZERO_TIME_FROM) || (RCtime > RC_ZERO_TIME_TO && RCtime < RC_MAX_TIME);
}

void setup() {
  Serial.begin(115200);

  enableInterrupts();

  pinMode(CRANE_FEEDBACK_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_COIL_1_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_COIL_2_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_COIL_3_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_COIL_4_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_A_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_B_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_A_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_B_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);

  leftHatch.attach(LEFT_HATCH_SERVO_PIN);
  rightHatch.attach(RIGHT_HATCH_SERVO_PIN);
  ledDoor.attach(LED_DOOR_PIN);
  enableLighting(false);
}

void loop() {

  if (Serial.available() > 0) {
    char received = Serial.read();

    if (received == 'r') {  //Radio controlled
      radioControlled = true;
      enableInterrupts();
    }
    else if (received == 's') { //Controlled through Serial
      Serial.println("OK");
      radioControlled = false;
      disableInterrupts();
    }
    else if (received == 'e' && !radioControlled) { //elevate the crane
      elevated = HIGH;
    }
    else if (received == 'd' && !radioControlled) { //drop the crane
      elevated = LOW;
    }
    else if (received == 'm' && !radioControlled) { //setting the speed of the motors
      long t = millis();
      motorSpeedBase = Serial.parseInt();
      motorSpeedChange = Serial.parseInt();
    }
    else if (received == 'b') { //checking the status of the battery
      Serial.println((int)(getBatteryVoltage() * 100));
    }
    else if (received == 'l') { //turn on lighting
      enableLighting(true);
    }
    else if (received == 'o') { //turn off lighting
      enableLighting(false);
    }
  }

  //in case of autonomous operation moving can be disabled remotely
  if (movingEnabled) {
    handleCrane();
    refreshLeftMotor();
    refreshRightMotor();
  }
  else {
    motorSpeedChange = 0;
    motorSpeedBase = 0;
    refreshLeftMotor();
    refreshRightMotor();
  }
}

/**
 * Enable or disable front lighting
 */
void enableLighting(bool enable) {
  if (enable) {
    ledDoor.write(LED_DOOR_OPEN);
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    ledDoor.write(LED_DOOR_CLOSED);
    digitalWrite(LED_PIN, LOW);
  }
}

/**
 * Moving the crane and thus the beer to the desired state
 */
void handleCrane() {
  int craneValue = analogRead(CRANE_FEEDBACK_PIN);

  if (elevated) {
    if (craneValue < CRANE_ELEVATED_VALUE) {
      elevateCrane();
    }
    else {
      disableTheCoils();
    }
    openTheHatch();
  }
  else {
    if (craneValue < CRANE_LOWERED_VALUE) {
      closeTheHatch();
      disableTheCoils();
    }
    else {
      dropCrane();
    }
  }
}

/**
 * Enabling interrupts
 */
void enableInterrupts() {
  attachInterrupt(digitalPinToInterrupt(RC_STATE_SWITCH_PIN), calcSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_HORIZONTAL_PIN), calcHorizontal, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_VERTICAL_PIN), calcVertical, CHANGE);
}

/**
 * Disabling interrupts. They don't need in case of autonomous operation.
 * RC_STATE_SWITCH_PIN stayed enabled because in autonomous mode it is used as a safety switch
 */
void disableInterrupts() {
  detachInterrupt(RC_HORIZONTAL_PIN);
  detachInterrupt(RC_VERTICAL_PIN);
  //detachInterrupt(RC_STATE_SWITCH_PIN);
}

/**
 * The hatch needs to be opened to elevate the crane
 */
void openTheHatch() {
  leftHatch.write(LEFT_SERVO_ZERO_ANGLE + HATCH_OPENED_ANGLE);
  rightHatch.write(RIGHT_SERVO_ZERO_ANGLE - HATCH_OPENED_ANGLE);
}

/**
 * Closing the hatch after the crane is lowered
 */
void closeTheHatch() {
  leftHatch.write(LEFT_SERVO_ZERO_ANGLE);
  rightHatch.write(RIGHT_SERVO_ZERO_ANGLE);
}

/**
 * Lowering the crane
 */
void dropCrane() {
  motorState++;
  if (motorState > 7) {
    motorState = 0;
  }
  moveMotorToState(motorState);
}

/**
 * Elevating the crane
 */
void elevateCrane() {
  motorState--;
  if (motorState < 0) {
    motorState = 7;
  }
  moveMotorToState(motorState);
}

/**
 * The coils of the stepper motor will be turned off to avoid overheating and reduce power consumption
 */
void disableTheCoils() {
  motorState = 8;
  moveMotorToState(motorState);
}

/**
 * Moving the stepper motor
 */
void moveMotorToState(int state) {
  switch (state) {
    case 0:
      motorStep(LOW, LOW, LOW, HIGH);
      break;
    case 1:
      motorStep(LOW, LOW, HIGH, HIGH);
      break;
    case 2:
      motorStep(LOW, LOW, HIGH, LOW);
      break;
    case 3:
      motorStep(LOW, HIGH, HIGH, LOW);
      break;
    case 4:
      motorStep(LOW, HIGH, LOW, LOW);
      break;
    case 5:
      motorStep(HIGH, HIGH, LOW, LOW);
      break;
    case 6:
      motorStep(HIGH, LOW, LOW, LOW);
      break;
    case 7:
      motorStep(HIGH, LOW, LOW, HIGH);
      break;
    default:
      motorStep(LOW, LOW, LOW, LOW);
      break;
  }
}

/**
 * switching on or off the coils of the stepper motor
 */
void motorStep(int coil1, int coil2, int coil3, int coil4) {
  digitalWrite(STEPPER_MOTOR_COIL_1_PIN, coil1);
  digitalWrite(STEPPER_MOTOR_COIL_2_PIN, coil2);
  digitalWrite(STEPPER_MOTOR_COIL_3_PIN, coil3);
  digitalWrite(STEPPER_MOTOR_COIL_4_PIN, coil4);
  delay(1);
}

/**
 * returning the calculated voltage of the battery
 */
float getBatteryVoltage() {
  return analogRead(BATTERY_PIN) * ADC_VALUE_TO_BAT_VOLTAGE_MULTIPLIER;
}

/**
 * update the speed of the left motor. It is calulcated from the
 * base speed of the tank and the sterring parameter
 */
void refreshLeftMotor() {
  int leftMotorSpeed = motorSpeedBase - motorSpeedChange;

  if (leftMotorSpeed > MAX_PWM_RESOLUTION) {
    leftMotorSpeed = MAX_PWM_RESOLUTION;
  }
  else if (leftMotorSpeed < -MAX_PWM_RESOLUTION) {
    leftMotorSpeed = -MAX_PWM_RESOLUTION;
  }

  if (leftMotorSpeed > MIN_SPEED) {
    digitalWrite(LEFT_MOTOR_A_PIN, LOW);
    digitalWrite(LEFT_MOTOR_B_PIN, HIGH);
  }
  else if (leftMotorSpeed < -MIN_SPEED) {
    digitalWrite(LEFT_MOTOR_A_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_B_PIN, LOW);
  }
  else {
    digitalWrite(LEFT_MOTOR_A_PIN, LOW);
    digitalWrite(LEFT_MOTOR_B_PIN, LOW);
  }
  analogWriteResolution(8);
  analogWrite(LEFT_MOTOR_SPEED_PIN, abs(leftMotorSpeed));
}

/**
 * update the speed of the right motor. It is calulcated from the
 * base speed of the tank and the sterring parameter
 */
void refreshRightMotor() {
  int rightMotorSpeed = motorSpeedBase + motorSpeedChange;

  if (rightMotorSpeed > MAX_PWM_RESOLUTION) {
    rightMotorSpeed = MAX_PWM_RESOLUTION;
  }
  else if (rightMotorSpeed < -MAX_PWM_RESOLUTION) {
    rightMotorSpeed = -MAX_PWM_RESOLUTION;
  }

  if (rightMotorSpeed > MIN_SPEED) {
    digitalWrite(RIGHT_MOTOR_A_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_B_PIN, HIGH);
  }
  else if (rightMotorSpeed < -MIN_SPEED) {
    digitalWrite(RIGHT_MOTOR_A_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);
  }
  else {
    digitalWrite(RIGHT_MOTOR_A_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);
  }
  analogWriteResolution(8);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, abs(rightMotorSpeed));
}

