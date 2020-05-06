#include <ESP32AsyncStepper.h>
#include <Arduino.h>

#define MOTOR1_STEP_PIN GPIO_NUM_23 //GPIO_NUM_2 = ESP32 Internal blue led
#define MOTOR1_DIRECTION_PIN GPIO_NUM_22
#define MOTOR2_STEP_PIN GPIO_NUM_19
#define MOTOR2_DIRECTION_PIN GPIO_NUM_21
#define MOTOR3_STEP_PIN GPIO_NUM_18
#define MOTOR3_DIRECTION_PIN GPIO_NUM_5
#define MOTOR4_STEP_PIN GPIO_NUM_2
#define MOTOR4_DIRECTION_PIN GPIO_NUM_4
#define MOTOR5_STEP_PIN GPIO_NUM_13
#define MOTOR5_DIRECTION_PIN GPIO_NUM_12
#define MOTOR6_STEP_PIN GPIO_NUM_14
#define MOTOR6_DIRECTION_PIN GPIO_NUM_27

ESP32AsyncStepper stepper1(MOTOR1_STEP_PIN, MOTOR1_DIRECTION_PIN, MCPWM_UNIT_0, MCPWM0A, MCPWM_TIMER_0);
ESP32AsyncStepper stepper2(MOTOR2_STEP_PIN, MOTOR2_DIRECTION_PIN, MCPWM_UNIT_0, MCPWM1A, MCPWM_TIMER_1);
ESP32AsyncStepper stepper3(MOTOR3_STEP_PIN, MOTOR3_DIRECTION_PIN, MCPWM_UNIT_0, MCPWM2A, MCPWM_TIMER_2);
ESP32AsyncStepper stepper4(MOTOR4_STEP_PIN, MOTOR4_DIRECTION_PIN, MCPWM_UNIT_1, MCPWM0A, MCPWM_TIMER_0);
ESP32AsyncStepper stepper5(MOTOR5_STEP_PIN, MOTOR5_DIRECTION_PIN, MCPWM_UNIT_1, MCPWM1A, MCPWM_TIMER_1);
ESP32AsyncStepper stepper6(MOTOR6_STEP_PIN, MOTOR6_DIRECTION_PIN, MCPWM_UNIT_1, MCPWM2A, MCPWM_TIMER_2);

bool continuous = false;
unsigned long previousmillis;

void setup() { 
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  Serial.println();
  Serial.println("START");
  stepper1.begin();
  stepper1.setSpeedInStepsPerSecond(200);
  stepper1.setAccelerationInStepsPerSecondPerSecond(20000);
  stepper2.begin();
  stepper2.setSpeedInStepsPerSecond(400);
  stepper2.setAccelerationInStepsPerSecondPerSecond(10000);
  stepper3.begin();
  stepper3.setSpeedInStepsPerSecond(600);
  stepper3.setAccelerationInStepsPerSecondPerSecond(5000);
  stepper4.begin();
  stepper4.setSpeedInStepsPerSecond(800);
  stepper4.setAccelerationInStepsPerSecondPerSecond(2500);
  stepper5.begin();
  stepper5.setSpeedInStepsPerSecond(1000);
  stepper5.setAccelerationInStepsPerSecondPerSecond(1000);
  stepper6.begin();
  stepper6.setSpeedInStepsPerSecond(1200);
  stepper6.setAccelerationInStepsPerSecondPerSecond(500);
}

void loop() {
  if (millis() - previousmillis > 2000) {
    previousmillis = millis();
    Serial.printf("current position:\nMOTOR 1\tMOTOR 2\tMOTOR 3\tMOTOR 4\tMOTOR 5\tMOTOR 6\n%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n",
      stepper1.getCurrentPositionInSteps(),
      stepper2.getCurrentPositionInSteps(),
      stepper3.getCurrentPositionInSteps(),
      stepper4.getCurrentPositionInSteps(),
      stepper5.getCurrentPositionInSteps(),
      stepper6.getCurrentPositionInSteps());
  }
  if (Serial.available()) {
    String command = Serial.readString();
    if (command.charAt(0) == '+') {
      Serial.println("Continuous fordward");
      stepper1.runContinuous(true, true);
      stepper2.runContinuous(true, true);
      stepper3.runContinuous(true, true);
      stepper4.runContinuous(true, true);
      stepper5.runContinuous(true, true);
      stepper6.runContinuous(true, true);
    }
    if (command.charAt(0) == '-') {
      Serial.println("Continuous backward");
      stepper1.runContinuous(true, false);
      stepper2.runContinuous(true, false);
      stepper3.runContinuous(true, false);
      stepper4.runContinuous(true, false);
      stepper5.runContinuous(true, false);
      stepper6.runContinuous(true, false);
    }
    if (command.charAt(0) == '0') {
      Serial.println("Continuous stop");
      stepper1.runContinuous(false, false);
      stepper2.runContinuous(false, false);
      stepper3.runContinuous(false, false);
      stepper4.runContinuous(false, false);
      stepper5.runContinuous(false, false);
      stepper6.runContinuous(false, false);
    }
    if (command.charAt(0) == 'G') { //EXAMPLE G0 G20000 G-20000 G3200
      int value = command.substring(1).toInt();
      Serial.printf("Go to absolute position: %d\n", value);
      stepper1.absoluteMoveInSteps(value);
      stepper2.absoluteMoveInSteps(value);
      stepper3.absoluteMoveInSteps(value);
      stepper4.absoluteMoveInSteps(value);
      stepper5.absoluteMoveInSteps(value);
      stepper6.absoluteMoveInSteps(value);
    }
    if (command.charAt(0) == 'R') { //EXAMPLE R0 R20000 R-20000
      int value = command.substring(1).toInt();
      Serial.printf("Go to relative position: %d\n", value);
      stepper1.relativeMoveInSteps(value);
      stepper2.relativeMoveInSteps(value);
      stepper3.relativeMoveInSteps(value);
      stepper4.relativeMoveInSteps(value);
      stepper5.relativeMoveInSteps(value);
      stepper6.relativeMoveInSteps(value);
    }
    if (command.charAt(0) == 'S') { //EXAMPLE SX 20000 Where X = motor
      int value = command.substring(3).toInt();
      Serial.printf("Setting motor%c speed: %d\n", command.charAt(1), value);
      switch (command.charAt(1)) {
        case '1': {
          stepper1.setSpeedInStepsPerSecond(value);
          break;
        }
        case '2': {
          stepper2.setSpeedInStepsPerSecond(value);
          break;
        }
        case '3': {
          stepper3.setSpeedInStepsPerSecond(value);
          break;
        }
        case '4': {
          stepper4.setSpeedInStepsPerSecond(value);
          break;
        }
        case '5': {
          stepper5.setSpeedInStepsPerSecond(value);
          break;
        }
        case '6': {
          stepper6.setSpeedInStepsPerSecond(value);
          break;
        }
      }
    }
    if (command.charAt(0) == 'A') { //EXAMPLE AX 20000 Where X = motor 
      int value = command.substring(3).toInt();
      Serial.printf("Setting motor%c acceleration: %d\n", command.charAt(1), value);
      switch (command.charAt(1)) {
        case '1': {
          stepper1.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
        case '2': {
          stepper2.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
        case '3': {
          stepper3.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
        case '4': {
          stepper4.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
        case '5': {
          stepper5.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
        case '6': {
          stepper6.setAccelerationInStepsPerSecondPerSecond(value);
          break;
        }
      }
    }
    if (command.charAt(0) == 'C') {
      Serial.println("toggle continuous movement"); 
      continuous = !continuous;
    }
  }
  if (continuous && stepper1.motionComplete() && stepper2.motionComplete() && stepper3.motionComplete()
     && stepper4.motionComplete() && stepper5.motionComplete() && stepper6.motionComplete()) {
    stepper1.relativeMoveInSteps(3200);
    stepper2.relativeMoveInSteps(3200);
    stepper3.relativeMoveInSteps(3200);
    stepper4.relativeMoveInSteps(3200);
    stepper5.relativeMoveInSteps(3200);
    stepper6.relativeMoveInSteps(3200);
  }
}
