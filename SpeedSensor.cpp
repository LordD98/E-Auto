#include "SpeedSensor.h"
#include "Arduino.h"

unsigned long SpeedSensor::MIN_INT = 15385;
extern double getAverage(unsigned long *buff, int len);

SpeedSensor::SpeedSensor(int speedSendPinIn) : speedSendPin(speedSendPinIn)
{

}

SpeedSensor::~SpeedSensor() {}

void SpeedSensor::setup()
{
	pinMode(speedSendPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(speedSendPin), speedInterrupt, RISING);  
}

/**
 * This function gets called by the main interrupt handler in E-Auto.ino
 */
void SpeedSensor::interrupt(unsigned long microseconds)
{
	//if(microseconds > MIN_INT)
	//	currentSpeed = 1000000.0 / (double)microseconds;
	
	static byte index = 0;
	
	if (microseconds > MIN_INT)		// if microseconds is smaller than MIN_INT
	{								// discard the value as it must be a glitch
		lastTurn = microseconds;
		rawValArray[index] = microseconds;
		index =	(index+1) % 8;
	}
}

/**
 * Returns current state
 * Use: ???
 */
bool SpeedSensor::getState()
{
	return state;
}

/**
 * return current speed
 */
double SpeedSensor::getSpeed()
{
	// An option here would be to check if micros()-lastmicros > lastTurn
	//		=> then returning 1e6/(micros()-lastmicros) or averaging with this value
	// or else returning the average as below
	// This would account for either really slow rotational speed or a still standing car
	// !Also, check if micros()-lastmicros is bigger than an even bigger value than 5e5 to define speed 0!

	if (micros() - lastMicros < 5e5)	// 5e5µs is about 0.5 turns/sec
	{
		currentSpeed = 1e6/getAverage(rawValArray, arraySize);	// Average
		//currentSpeed = 1e6 / lastTurn;						// Instantaneous
		return currentSpeed;
	}
	else
	{
		return 0.0;
	}
}

