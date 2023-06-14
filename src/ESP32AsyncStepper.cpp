//
  //      ******************************************************************
  //      *                                                                *
  //      *               ESP32 Async Stepper Motor Driver                 *
  //      *                                                                *
  //      *                           pikolo84                             *
  //      *                                                                *
  //      *                                                                *
  //      ******************************************************************


  // MIT License
  // 
  // Copyright (c) 2014 Stanley Reifel & Co.
  // 
  // Permission is hereby granted, free of charge, to any person obtaining a copy
  // of this software and associated documentation files (the "Software"), to deal
  // in the Software without restriction, including without limitation the rights
  // to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  // copies of the Software, and to permit persons to whom the Software is furnished
  // to do so, subject to the following conditions:
  // 
  // The above copyright notice and this permission notice shall be included in all
  // copies or substantial portions of the Software.
  // 
  // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  // OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  // SOFTWARE.

#ifndef ESP32AsyncStepper_cpp
#define ESP32AsyncStepper_cpp
#include "ESP32AsyncStepper.h"

ESP32AsyncStepper::ESP32AsyncStepper(gpio_num_t stepPinNumber, gpio_num_t directionPinNumber, 
      mcpwm_unit_t mcpwmUnit, mcpwm_io_signals_t mcpwmIoSignalsPwm, mcpwm_timer_t mcpwmTimer) { //Optional parameters if only 1 motor
  _stepPin = stepPinNumber;
  _directionPin = directionPinNumber;
  _mcpwmIoSignalsPwm = mcpwmIoSignalsPwm;
  _mcpwmUnit = mcpwmUnit;
  _mcpwmTimer = mcpwmTimer;
  pinMode(_stepPin, OUTPUT);
  pinMode(_directionPin, OUTPUT);
}

void ESP32AsyncStepper::begin() {
  _motorStatus = STEPPER_MOTOR_STOPPED;
  //MCPWM
  mcpwm_gpio_init(_mcpwmUnit, _mcpwmIoSignalsPwm, _stepPin);
  mcpwm_config_t pwm_config;
    pwm_config.frequency = MCPWM_MIN_FREQUENCY;
    pwm_config.cmpr_a = 0.0;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER; 
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_mcpwmUnit, _mcpwmTimer, &pwm_config);
  //MCPWM
  attachInterruptArg(_stepPin, isr_handler, (void*)this, RISING);

  pinMode(_directionPin, OUTPUT);
  //FIX SHARED PIN PROBLEM
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[_stepPin]);

  _desiredSpeedInStepsPerSecond = 200.0;
  _accelerationInStepsPerSecondPerSecond = 200;
}

void ESP32AsyncStepper::stopMotor() {
  if (_motorStatus == STEPPER_MOTOR_STOPPED) return;
  mcpwm_set_duty_in_us(_mcpwmUnit, _mcpwmTimer, MCPWM_OPR_A, 0);
  mcpwm_set_signal_low(_mcpwmUnit, _mcpwmTimer, MCPWM_OPR_A);
  _motorStatus = STEPPER_MOTOR_STOPPED;
  delay(10);
  _targetPositionInSteps = _currentPositionInSteps;
}

asyncStepperStatus_t ESP32AsyncStepper::getMotorStatus() {
  return _motorStatus;
}

// // Set position of the motor, this does not move the motor
// // Note: This function should only be called when the motor is stopped
// //    Enter:  currentPositionInSteps = the new position of the motor in steps
void ESP32AsyncStepper::SetPositionInSteps(long currentPositionInSteps) {
  _currentPositionInSteps = currentPositionInSteps;
}

// // ---------------------------------------------------------------------------------
// //                        Functions with units in steps 
// // ---------------------------------------------------------------------------------

// // get the current position of the motor in steps, this functions is updated
// // while the motor moves
// //  Exit:  a signed motor position in steps returned
long ESP32AsyncStepper::getCurrentPositionInSteps()
{
  return(_currentPositionInSteps);
}

void ESP32AsyncStepper::setSpeedInStepsPerSecond(float speedInStepsPerSecond) {
  _desiredSpeedInStepsPerSecond = speedInStepsPerSecond;
}

void ESP32AsyncStepper::setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond) {
  _accelerationInStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}

void ESP32AsyncStepper::relativeMoveInSteps(long distanceToMoveInSteps) {
  absoluteMoveInSteps(_currentPositionInSteps + distanceToMoveInSteps);
}

void ESP32AsyncStepper::absoluteMoveInSteps(long absolutePositionToMoveToInSteps) {
  if (_motorStatus != STEPPER_MOTOR_STOPPED) return;
  if (absolutePositionToMoveToInSteps == _currentPositionInSteps) return;
  // save the target location
  _targetPositionInSteps = absolutePositionToMoveToInSteps;
  xTaskCreate(taskProcessMovement, "processMovement", 4096, (void*)this, 2, NULL);
}

// // ---------------------------------------------------------------------------------
// //                        Continuous movement functions 
// // ---------------------------------------------------------------------------------

void ESP32AsyncStepper::runContinuous(bool run, bool direction) {
  if (run) {
    if (_motorStatus == STEPPER_MOTOR_STOPPED) {
      if (direction) {
        digitalWrite(_directionPin, HIGH);
        _direction = true;
      }
      else {
        digitalWrite(_directionPin, LOW);
        _direction = false;
      }
      _targetPositionInSteps = _currentPositionInSteps;
      xTaskCreate(taskProcessContinuousMovement, "processContinuousMovement", 1024, (void*)this, 2, NULL);
    }
  }
  else {
    if (_motorStatus != STEPPER_MOTOR_STOPPED)
      _motorStatus = STEPPER_MOTOR_STOPPING;
  }
}

//Private functions

void ESP32AsyncStepper::taskProcessMovement (void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  ESP32AsyncStepper *args = static_cast<ESP32AsyncStepper*>(pvParameters);
  long distanceToTravelInSteps = args->_targetPositionInSteps - args->_currentPositionInSteps;;
  double accelerationStep = args->_accelerationInStepsPerSecondPerSecond / (configTICK_RATE_HZ / xFrequency);
  args->_decelerationDistanceInSteps = (args->_desiredSpeedInStepsPerSecond * args->_desiredSpeedInStepsPerSecond) 
    / (2 * args->_accelerationInStepsPerSecondPerSecond);
    
  double currentSpeedRamp = 0.0;

    // determine the distance and direction to travel
  if (distanceToTravelInSteps < 0) {
    distanceToTravelInSteps = -distanceToTravelInSteps;
    digitalWrite(args->_directionPin, LOW);
    args->_direction = false;
    if (args->_decelerationDistanceInSteps > (distanceToTravelInSteps / 2)) {
      args->_decelerationDistanceInSteps = distanceToTravelInSteps / 2;
    }
    args->_decelerationDistanceInSteps = args->_targetPositionInSteps + args->_decelerationDistanceInSteps;
  }
  else {
    digitalWrite(args->_directionPin, HIGH);
    args->_direction = true;
    if (args->_decelerationDistanceInSteps > (distanceToTravelInSteps / 2)) {
      args->_decelerationDistanceInSteps = distanceToTravelInSteps / 2;
    }
    args->_decelerationDistanceInSteps = args->_targetPositionInSteps - args->_decelerationDistanceInSteps;
  }

  mcpwm_set_duty_type(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_frequency(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_MIN_FREQUENCY);
  args->_motorStatus = STEPPER_MOTOR_ACCELERATING;

  while(args->_currentPositionInSteps != args->_targetPositionInSteps) {
    if (args->_motorStatus == STEPPER_MOTOR_ACCELERATING) {
      currentSpeedRamp += accelerationStep;
      if (currentSpeedRamp >= args->_desiredSpeedInStepsPerSecond) {
        currentSpeedRamp = args->_desiredSpeedInStepsPerSecond;
        args->_motorStatus = STEPPER_MOTOR_AT_SPEED;
      }
    }
    else {
      if (args->_motorStatus == STEPPER_MOTOR_DECELERATING) {
        long remainigSteps = abs(args->_targetPositionInSteps - args->_currentPositionInSteps);
        currentSpeedRamp = sqrt(2*args->_accelerationInStepsPerSecondPerSecond*remainigSteps);
        if (currentSpeedRamp <= MCPWM_MIN_FREQUENCY) {
          currentSpeedRamp = MCPWM_MIN_FREQUENCY;
          args->_motorStatus = STEPPER_MOTOR_STOPPING;
        }
      }
    }
    if (currentSpeedRamp >= MCPWM_MIN_FREQUENCY) {
      args->_frequency = currentSpeedRamp;
      mcpwm_set_duty_in_us(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_US);
      mcpwm_set_frequency(args->_mcpwmUnit, args->_mcpwmTimer, args->_frequency);
    }
    xTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void ESP32AsyncStepper::taskProcessContinuousMovement (void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  ESP32AsyncStepper *args = static_cast<ESP32AsyncStepper*>(pvParameters);
  double accelerationStep;
  double currentSpeedRamp = 0.0;
  mcpwm_set_duty_type(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_frequency(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_MIN_FREQUENCY);
  args->_motorStatus = STEPPER_MOTOR_ACCELERATING;

  while(args->_motorStatus != STEPPER_MOTOR_STOPPED) {
    accelerationStep = args->_accelerationInStepsPerSecondPerSecond / (configTICK_RATE_HZ / xFrequency);
    double setSpeed;
    if (args->_motorStatus == STEPPER_MOTOR_STOPPING) {
      setSpeed = 0.0; 
      if (currentSpeedRamp <= MCPWM_MIN_FREQUENCY) {
        args->_motorStatus = STEPPER_MOTOR_STOPPED;
        mcpwm_set_duty_in_us(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, 0);
        mcpwm_set_signal_low(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A);
      }
    }
    else {
      setSpeed = args->_desiredSpeedInStepsPerSecond;
    }
      
    if (currentSpeedRamp < setSpeed) {
      currentSpeedRamp += accelerationStep;
      if (currentSpeedRamp > setSpeed) {
        currentSpeedRamp = setSpeed;
      }
    }
    if (currentSpeedRamp > setSpeed) {
      currentSpeedRamp -= accelerationStep;
      if (currentSpeedRamp <= setSpeed) {
        currentSpeedRamp = setSpeed;
      }
    }

    if (currentSpeedRamp >= MCPWM_MIN_FREQUENCY) {
      args->_frequency = currentSpeedRamp;
      mcpwm_set_duty_in_us(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_US);
      mcpwm_set_frequency(args->_mcpwmUnit, args->_mcpwmTimer, args->_frequency);
    }
    xTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void IRAM_ATTR ESP32AsyncStepper::isr_handler (void *arg) {
  ESP32AsyncStepper *args = static_cast<ESP32AsyncStepper*>(arg);
  if (args->_direction)
    args->_currentPositionInSteps +=1;
  else
    args->_currentPositionInSteps -=1;
  if (args->_currentPositionInSteps == args->_targetPositionInSteps) {
    mcpwm_set_duty_in_us(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A, 0);
    mcpwm_set_signal_low(args->_mcpwmUnit, args->_mcpwmTimer, MCPWM_OPR_A);
    args->_motorStatus = STEPPER_MOTOR_STOPPED;
  }
  else {
    if (args->_currentPositionInSteps == args->_decelerationDistanceInSteps)
      args->_motorStatus = STEPPER_MOTOR_DECELERATING;
  }
}

// // check if the motor has competed its move to the target position
// //  return:  true returned if the stepper is at the target position
bool ESP32AsyncStepper::motionComplete() {
  return (_currentPositionInSteps == _targetPositionInSteps);
}

#endif // ESP32AsyncStepper_cpp