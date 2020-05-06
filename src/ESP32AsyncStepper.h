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


#ifndef ESP32AsyncStepper_h
#define ESP32AsyncStepper_h

#include <Arduino.h>
#include <stdlib.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h" 
#include "soc/mcpwm_reg.h"

#define MCPWM_MIN_FREQUENCY 16
#define MCPWM_DUTY_US 2

#define PROCESS_MOVEMENT_PERIOD 10
#if PROCESS_MOVEMENT_PERIOD < 1
  #error PROCESS_MOVEMENT_PERIOD "PROCESS_MOVEMENT_PERIOD CAN'T BE LESS THAN 1"
#endif

typedef enum {
  STEPPER_MOTOR_STOPPED,
  STEPPER_MOTOR_ACCELERATING,
  STEPPER_MOTOR_AT_SPEED,
  STEPPER_MOTOR_DECELERATING,
  STEPPER_MOTOR_STOPPING
} asyncStepperStatus_t;

class ESP32AsyncStepper
{
  public:
    ESP32AsyncStepper(gpio_num_t stepPinNumber, gpio_num_t directionPinNumber, 
      mcpwm_unit_t mcpwmUnit = MCPWM_UNIT_0, mcpwm_io_signals_t mcpwmIoSignals = MCPWM0A, mcpwm_timer_t mcpwmTimer = MCPWM_TIMER_0);
    void begin();  

    //STEPS functions
    void SetPositionInSteps(long currentPositionInSteps);
    long getCurrentPositionInSteps();
    void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
    void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
    void relativeMoveInSteps(long distanceToMoveInSteps);
    void absoluteMoveInSteps(long absolutePositionToMoveToInSteps);

    //Continuous moving functions
    void runContinuous(bool run, bool direction);

    bool motionComplete();


  private:
    static void IRAM_ATTR isr_handler (void *arg);
    static void taskProcessMovement (void *arg);
    static void taskProcessContinuousMovement (void *arg);

    gpio_num_t _stepPin;
    gpio_num_t _directionPin;
    mcpwm_unit_t _mcpwmUnit;
    mcpwm_io_signals_t _mcpwmIoSignalsPwm;
    mcpwm_timer_t _mcpwmTimer;
    asyncStepperStatus_t _motorStatus;
    bool _direction;
    double _desiredSpeedInStepsPerSecond;
    double _accelerationInStepsPerSecondPerSecond;
    long _decelerationDistanceInSteps;
    long _targetPositionInSteps;
    uint32_t _frequency;
    volatile long _currentPositionInSteps = 0;
};
#endif //ESP32AsyncStepper_h

