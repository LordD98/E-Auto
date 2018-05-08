#include "SpeedSensor.h"
#include "Arduino.h"

SpeedSensor::SpeedSensor(int speedSendPinIn)
{

}

SpeedSensor::~SpeedSensor() {}

void SpeedSensor::setup()
{
	pinMode(SpeedSensor::speedSendPin, INPUT);
	//attachInterrupt(digitalPinToInterrupt(SpeedSensor::speedSendPin), , );  
}

void SpeedSensor::interrupt(unsigned long now)
{

}

bool SpeedSensor::getState()
{

}

double SpeedSensor::getSpeed()
{

}

