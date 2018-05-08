#include "DirectionControl.h"
#include "Arduino.h"



DirectionControl::DirectionControl(int servoPinIn, directionParameterSet directionParameters, pidParameterSet pidParameters)
	: controller(pidParameters)
{

}

DirectionControl::~DirectionControl() {}

void DirectionControl::setup()
{
	DirectionControl::directionServo.attach(DirectionControl::servoPin); // initialize servodirectionServo.attach(pin_servo); // initialize
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

