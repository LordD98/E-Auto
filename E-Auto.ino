
#include <string.h>
#include <math.h>
#include "DirectionControl.h"
#include "SpeedSensor.h"
#include "motorControl.h"

// pin declaration
// do NOT use DigitalOut 0 & 1 (reserved for Serial)
const int pin_servo = 0;			//pin for steering control 
const int pin_motor = 0;			//pin for motor control
const int pin_deadManSwitch = 0;	//dead man swich pin
const int pin_spuleLeft = 0;		//Left resonant circuit
const int pin_spuleRight = 0;		//Right resonant circuit
const int pin_speedSensor = 0;		//Input signal for speed sensor signal


//init global variables and objects
const int directionServoCenter = 0;//Center level for directionServo
const int directionMaxValue = 0;
const int directionMinValue = 0;
const int ofTrackDetectionLevel = 0; // minValue for off-track detection
DirectionControl directionControl = DirectionControl(pin_servo,{directionMaxValue,directionMinValue,directionServoCenter},{0, 0, 0, 0, directionServoCenter, 0, directionMinValue, directionMaxValue});


const double idleSpeed = 0;//Speed setpoint for idle
MotorControl motorControl(pin_deadManSwitch,pin_motor,{0,0,0,0,0,0,0,0});


SpeedSensor speedSens(pin_speedSensor);


void setup()
{
	staticInitCode();	//this function call must not be removed
	
	//debugging
	Serial.begin(9600); // for serial logging
	pinMode(LED_BUILTIN, OUTPUT);  // declare onboard-led (indicates "on track")
	
	directionControl.testServo();
}

/**
 * Main loop
 */
void loop()
{


   
}

void speedInterrupt()
{

}


int getAverage(int pin)
{

}

/**
 * The following code must not be changed
 */
void staticInitCode()
{
	pinMode(pin_deadManSwitch, INPUT_PULLUP);	//Set pullup for dead man switch
}


