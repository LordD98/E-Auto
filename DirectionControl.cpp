
#include "DirectionControl.h"
#include "Arduino.h"


DirectionControl::DirectionControl (
	int servoPinIn, directionParameterSet directionParameters, pidParameterSet pidParameters)
	: controller(pidParameters), servoPin(servoPinIn)
{

}

DirectionControl::~DirectionControl() {}

void DirectionControl::setup()
{
	directionServo.attach(servoPin);
}


void DirectionControl::updateController(double is)
{

}

int DirectionControl::getDirection()
{

}

void DirectionControl::updateDirection()
{

}

void DirectionControl::testServo()
{

}

