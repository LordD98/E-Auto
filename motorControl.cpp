#include "motorControl.h"
#include "Arduino.h"

MotorControl::MotorControl(unsigned int deadManSwitchPinIn,unsigned int motorPinIn,pidParameterSet pidParameters)
	: controller(pidParameters), deadManSwitchPin(deadManSwitchPinIn), motorPin(motorPinIn) 
{ }

/**
 * This function must not be changed
 */
void MotorControl::setup()
{
	pinMode(motorPin, OUTPUT);
	setState(MOTOR_STATES::STOP);
}


/**
 * Set the new wanted speed
 * speedSetpoint = Drehzahl
 */
void MotorControl::setSpeed(double speedSetpoint)
{
	controller.setSoll(speedSetpoint);
}

/**
 * Update the controller with the new current speed
 */
void MotorControl::updateController(double currentSpeed)
{
	motorDuty = (int)round(controller.calcOutput(currentSpeed));
}

/**
 * This function must not be changed
 */
void MotorControl::updateMotor(void)
{
	if(motorState == MOTOR_STATES::RUN  && !digitalRead(deadManSwitchPin))
	{
		analogWrite(motorPin, motorDuty);
	}
	else
	{
		analogWrite(motorPin, 0);	//Stop motor
	}
}

/**
 * This function must not be changed
 */
void MotorControl::setState(MOTOR_STATES stateIn)
{
	if(motorState != stateIn)
	{
		motorState = stateIn;
		if(motorState == MOTOR_STATES::RUN)
		{
			controller.resetIntegrator();
		}
		updateMotor();	//update Motor in case of a state change
	}
}

void MotorControl::setDuty(int duty)
{
	motorDuty = duty;
}

