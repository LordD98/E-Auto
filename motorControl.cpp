#include "motorControl.h"
#include "Arduino.h"

MotorControl::MotorControl(unsigned int deadManSwitchPinIn, unsigned int motorPinIn, pidParameterSet pidParameters)
	: controller(pidParameters)
{

}

/**
 * This function must not be changed
 */
void MotorControl::setup()
{
	pinMode(MotorControl::motorPin, OUTPUT);
	MotorControl::setState(MOTOR_STATES::STOP);
}

void MotorControl::setSpeed(double speedSetpoint)
{

}

void MotorControl::updateController(double currentSpeed)
{

}

/**
 * This function must not be changed
 */
void MotorControl::updateMotor(void)
{
	if(MotorControl::motorState==MOTOR_STATES::RUN && !digitalRead(MotorControl::deadManSwitchPin))
	{
		analogWrite(MotorControl::motorPin, MotorControl::motorDuty);
	}
	else
	{
		analogWrite(MotorControl::motorPin, 0);	//Stop motor
	}
}

/**
 * This function must not be changed
 */
void MotorControl::setState(MOTOR_STATES stateIn)
{
	if(MotorControl::motorState != stateIn)
	{
		MotorControl::motorState = stateIn;
		if(MotorControl::motorState==MOTOR_STATES::RUN)
		{
			MotorControl::controller.resetIntegrator();
		}
		MotorControl::updateMotor();	//update Motor in case of a state change
	}
}

