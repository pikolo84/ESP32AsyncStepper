#include <ESP32AsyncStepper.h>
#include <Arduino.h>

#define MOTOR_STEP_PIN GPIO_NUM_2 //GPIO_NUM_2 = ESP32 Internal blue led
#define MOTOR_DIRECTION_PIN GPIO_NUM_22

ESP32AsyncStepper stepper(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

bool continuous = false;
unsigned long previousmillis;

void setup() { 
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  Serial.println();
  Serial.println("START");
  stepper.begin();
  stepper.setSpeedInStepsPerSecond(3200);
  stepper.setAccelerationInStepsPerSecondPerSecond(20000);
}

void loop() {
  if (millis() - previousmillis > 2000) {
    previousmillis = millis();
    Serial.printf("current position: %i\n", stepper.getCurrentPositionInSteps());
  }
  if (Serial.available()) {
    String command = Serial.readString();
    if (command.charAt(0) == '+') {
      Serial.println("Continuous fordward");
      stepper.runContinuous(true, true);
    }
    if (command.charAt(0) == '-') {
      Serial.println("Continuous backward");
      stepper.runContinuous(true, false);
    }
    if (command.charAt(0) == '0') {
      Serial.println("Continuous stop");
      stepper.runContinuous(false, false);
    }
    if (command.charAt(0) == 'G') { //EXAMPLE G0 G20000 G-20000 G3200
      int value = command.substring(1).toInt();
      Serial.printf("Go to absolute position: %d\n", value);
      stepper.absoluteMoveInSteps(value);
    }
    if (command.charAt(0) == 'R') { //EXAMPLE R0 R20000 R-20000
      int value = command.substring(1).toInt();
      Serial.printf("Go to relative position: %d\n", value);
      stepper.relativeMoveInSteps(value);
    }
    if (command.charAt(0) == 'S') { //EXAMPLE S20000
      int value = command.substring(1).toInt();
      Serial.printf("Setting speed: %d\n", value);
      stepper.setSpeedInStepsPerSecond(value);
    }
    if (command.charAt(0) == 'A') { //EXAMPLE A10000
      int value = command.substring(1).toInt();
      Serial.printf("Setting acceleration: %d\n", value);
      stepper.setAccelerationInStepsPerSecondPerSecond(value);
    }
    if (command.charAt(0) == 'C') {
      Serial.println("toggle continuous movement");
      continuous = !continuous;
    }
  }
  if (continuous && stepper.motionComplete()) {
    stepper.relativeMoveInSteps(3200);
  }
}
