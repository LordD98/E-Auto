#pragma once

#include "PID.h"

enum class MOTOR_STATES{ STOP, RUN };

class MotorControl
{
  public:
    
    MotorControl(unsigned int, unsigned int, pidParameterSet);
    void setSpeed(double);
    void updateMotor(void);
    void setState(MOTOR_STATES);
    void updateController(double);
    void setup();

	void setDuty(int duty);	// Only for testing
    
  private:
    unsigned int motorPin;
    unsigned int deadManSwitchPin;
    int motorDuty;
    PID controller;
    MOTOR_STATES motorState = MOTOR_STATES::STOP;
};

