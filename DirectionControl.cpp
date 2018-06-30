
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


void DirectionControl::updateController(int is)	// is € [-1023, 1023]
{
	double new_direction;
	new_direction = controller.calcOutput(static_cast<double>(is) / 1.0);
	
	currentDirection = static_cast<int>(new_direction);
}

int DirectionControl::getDirection()
{
	return currentDirection;
}

void DirectionControl::updateDirection()
{
	Serial.println(currentDirection);
	directionServo.write(currentDirection);
}

void DirectionControl::testServo()
{
	//const double T = 100000000.0;
	//double reltime = fmod(micros() - startMicros, T);
	//if (reltime < T / 2)
	//{
	//	//motorControl.setDuty(150.0*reltime / T);
	//	motorControl.setSpeed(30.0* reltime / T + 10);
	//}
	//else
	//{
	//	//motorControl.setDuty(150.0* (T-reltime) / T);
	//	motorControl.setSpeed(30.0* (T-reltime) / T + 10);
	//}


	const long T = 5000000;									// Triangle wave
	int pos;												// with Period T = 2s,
	//long reltime = (micros() - startMicros) % T;			// amplitude between [0, 180]
	//if (reltime < T / 2)									//
	//{														//
	//	pos = map(reltime, 0, T/2, 0, 180);					//
	//}														//
	//else													//
	//{														//
	//	pos = map(T-reltime, 0, T/2, 0, 180);				//
	//}

	//
	//	Gerade bei pos = 101s
	//

	directionServo.write(-70);
	delay(100);
	directionServo.write(70);
	delay(100);
	//pos = map(analogRead(pin_test), 0, 1023, 0, 180);
	pos = 101;
	Serial.println("Servo Test!");
	directionServo.write(pos);
}

