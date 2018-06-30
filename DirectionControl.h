#pragma once

#include "PID.h"
#include <Servo.h>

struct directionParameterSet
{
  int maxPos, minPos, centerPos;
};

class DirectionControl
{
public:
	DirectionControl(int,directionParameterSet,pidParameterSet);
	~DirectionControl();
	
	int getDirection();
	
	void updateDirection();
	void updateController(int);
	void testServo();
	void setup();
	Servo directionServo;
	
private:
	PID controller;
	
	int servoPin;
	int centerPos;	// ~= 0
	int maxPos;		// Werte an der Spule
	int minPos;		// Werte an der Spule
	
	int currentDirection;	// left - right
};

