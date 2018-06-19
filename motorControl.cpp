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

void MotorControl::softStart(double speed, SpeedSensor *sens)
{
	controller.setSoll(speed);
	double currentSpeed = sens->getSpeed();
	for (int i = 0; i < 255 && currentSpeed < speed; i++)
	{
		setDuty(i);
		updateMotor();
		//controller.resetIntegrator();
		controller.calcOutput(currentSpeed);
		//Serial.print("i: ");
		//Serial.println(i);
		//Serial.print("Speed: ");
		//Serial.println(currentSpeed);
		currentSpeed = sens->getSpeed();
		delay(5);
	}
}


/**
 * Set the new wanted speed
 * speedSetpoint =^= rotational speed in turns/sec
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
	if (digitalRead(deadManSwitchPin))		// If the car stands still (due to deadman switch),
	{										// reset the integral.
		controller.resetIntegrator();		// This prevents a rough starting of the car
	}										// due to old integral values
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

/**
 * Manual override of the PID-output values
 */
void MotorControl::setDuty(int duty)
{
	motorDuty = duty;
}

