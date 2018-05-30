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
	
	static uint8_t index = 0;
	
	if (microseconds > MIN_INT)
	{
		lastTurn = microseconds;
		rawValArray[index] = microseconds;
		index++;
		index %= 8;
	}
}

/**
 * ???
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
	if (micros() - lastMicros < 5e5)
	{
		currentSpeed = 1e6/getAverage(rawValArray, arraySize);	// Average
		//currentSpeed = 1e6 / lastTurn;						// Instantaneous
		return currentSpeed;
	}
	else
	{
		return 0;
	}
}

