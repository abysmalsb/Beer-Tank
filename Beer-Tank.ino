#include <Servo.h>

/** Connection settings **/
#define LED_PIN A0
#define BATTERY_PIN A1
#define CRANE_FEEDBACK_PIN A2
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

int motorState = 8; // the motor is switched off
int elevated = 1;

boolean radioControlled = true;

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
      elevated = state_switch_pulse_time < 1500;
    }
  }
}

boolean timeValid(int RCtime) {
  return (RCtime > RC_MIN_TIME && RCtime < RC_ZERO_TIME_FROM) || (RCtime > RC_ZERO_TIME_TO && RCtime < RC_MAX_TIME);
}

void setup() {
  Serial.begin(115200);

  enableInterrupts();

  pinMode(CRANE_FEEDBACK_PIN, INPUT);
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
}

void loop() {

  if (Serial.available() > 0) {
    char received = Serial.read();

    if (received == 'r') {  //Radio controlled
      radioControlled = true;
      enableInterrupts();
    }
    else if (received == 's') { //Controlled through Serial
      radioControlled = false;
      disableInterrupts();
    }
    else if (received == 'e' && !radioControlled){  //elevate crane
      elevated = HIGH;
    }
    else if (received == 'd' && !radioControlled){  //drop crane
      elevated = LOW;
    }
    else if (received == 'm' && !radioControlled){  //setting the speed of the motors
      long t = millis();
      motorSpeedBase = Serial.parseInt();
      motorSpeedChange = Serial.parseInt();
    }
    else if (received == 'b'){
      Serial.println((int)(getBatteryVoltage() * 100));
    }
  }

  handleCrane();
  refreshLeftMotor();
  refreshRightMotor();
}

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

void enableInterrupts() {
  attachInterrupt(digitalPinToInterrupt(RC_HORIZONTAL_PIN), calcHorizontal, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_VERTICAL_PIN), calcVertical, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STATE_SWITCH_PIN), calcSwitch, CHANGE);
}

void disableInterrupts() {
  detachInterrupt(RC_HORIZONTAL_PIN);
  detachInterrupt(RC_VERTICAL_PIN);
  detachInterrupt(RC_STATE_SWITCH_PIN);
}

void openTheHatch() {
  leftHatch.write(LEFT_SERVO_ZERO_ANGLE + HATCH_OPENED_ANGLE);
  rightHatch.write(RIGHT_SERVO_ZERO_ANGLE - HATCH_OPENED_ANGLE);
}

void closeTheHatch() {
  leftHatch.write(LEFT_SERVO_ZERO_ANGLE);
  rightHatch.write(RIGHT_SERVO_ZERO_ANGLE);
}

void dropCrane() {
  motorState++;
  if (motorState > 7) {
    motorState = 0;
  }
  moveMotorToState(motorState);
}

void elevateCrane() {
  motorState--;
  if (motorState < 0) {
    motorState = 7;
  }
  moveMotorToState(motorState);
}

//The coils of the motor will be turned off to avoid overheating and reduce power consumption
void disableTheCoils() {
  motorState = 8;
  moveMotorToState(motorState);
}

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

void motorStep(int coil1, int coil2, int coil3, int coil4) {
  digitalWrite(STEPPER_MOTOR_COIL_1_PIN, coil1);
  digitalWrite(STEPPER_MOTOR_COIL_2_PIN, coil2);
  digitalWrite(STEPPER_MOTOR_COIL_3_PIN, coil3);
  digitalWrite(STEPPER_MOTOR_COIL_4_PIN, coil4);
  delay(1);
}

float getBatteryVoltage() {
  return analogRead(BATTERY_PIN) * ADC_VALUE_TO_BAT_VOLTAGE_MULTIPLIER;
}

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
    Serial.print(horizontal_pulse_time);
    Serial.print(" ");
    Serial.println(vertical_pulse_time);
  }
  else if (leftMotorSpeed < -MIN_SPEED) {
    digitalWrite(LEFT_MOTOR_A_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_B_PIN, LOW);
    Serial.print(horizontal_pulse_time);
    Serial.print(" ");
    Serial.println(vertical_pulse_time);
  }
  else {
    digitalWrite(LEFT_MOTOR_A_PIN, LOW);
    digitalWrite(LEFT_MOTOR_B_PIN, LOW);
  }
  analogWriteResolution(8);
  analogWrite(LEFT_MOTOR_SPEED_PIN, abs(leftMotorSpeed));
}

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
    Serial.print(horizontal_pulse_time);
    Serial.print(" ");
    Serial.println(vertical_pulse_time);
  }
  else if (rightMotorSpeed < -MIN_SPEED) {
    digitalWrite(RIGHT_MOTOR_A_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);
    Serial.print(horizontal_pulse_time);
    Serial.print(" ");
    Serial.println(vertical_pulse_time);
  }
  else {
    digitalWrite(RIGHT_MOTOR_A_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);
  }
  analogWriteResolution(8);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, abs(rightMotorSpeed));
}

