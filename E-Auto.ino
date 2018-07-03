
#include <Servo.h>
#include <string.h>
#include <math.h>
#include "DirectionControl.h"
#include "SpeedSensor.h"
#include "motorControl.h"

// pin declaration
// do NOT use DigitalOut 0 & 1 (reserved for Serial)
const int pin_servo = 5;			//pin for steering control 
const int pin_motor = 3;			//pin for motor control
const int pin_deadManSwitch =  8;	//dead man swich pin
const int pin_spuleLeft = A1;		//Left resonant circuit
const int pin_spuleRight = A0;		//Right reso nant circuit
const int pin_speedSensor = 2;		//Input signal for speed sensor signal; Pin2: INT0


//init global variables and objects
const int directionCenter = -150;
const int directionMaxValue = 400;
const int directionMinValue = -400;

const int directionServoCenter = 101;	//Center level for directionServo
const int directionServoMaxValue = 171;	// 101+70 = 171
const int directionServoMinValue = 31;	// actually 101-70 = 31
const int ofTrackDetectionLevel = 0;// minValue for off-track detection

DirectionControl directionControl = DirectionControl(
	pin_servo,																	// Servo-Pin
	{directionMaxValue,directionMinValue,directionCenter},						// directionParamSet
	{ 0.04, 0.0, 0.0109375, directionCenter, directionServoCenter, 0, directionServoMinValue, directionServoMaxValue}		// pidParamSet
	//	| 0.0525 stable	  p = (1.0)* 70.0/400.0 * 0.5 => Schwingung
);

const double idleSpeed = 0;			//Speed setpoint for idle
MotorControl motorControl = MotorControl(pin_deadManSwitch, pin_motor, {2.502, 6.54118, 0, 10.0, 0, 0, 5, 253});	// Works well
// These values are also okay: {5.0, 5.0, 0, 0, 0, 0, 5, 253} 

SpeedSensor speedSens = SpeedSensor(pin_speedSensor);

unsigned long lastMicros = 0;

int rawValArray[8];


void setup()
{
	staticInitCode();	//this function call must not be removed
	//debugging
	Serial.begin(9600); // for serial logging
	pinMode(LED_BUILTIN, OUTPUT);  // declare onboard-led (indicates "on track")

	directionControl.setup();
	//directionControl.testServo();


	motorControl.setup();
	speedSens.setup();

	motorControl.setState(MOTOR_STATES::RUN);	// Start motor
	//motorControl.softStart(10.0, &speedSens);
}

/**
 * Main loop
 */
void loop()
{
	static unsigned long startMicros = micros();

	static byte index = 0;									// Direction-Sensor
	int sensor_left = analogRead(pin_spuleLeft);			// Werte auslesen
	int sensor_right = analogRead(pin_spuleRight);			//
	rawValArray[index] = sensor_left-sensor_right;			//
	index = (index + 1)  % 8;								//
	int average = getAverageInt(rawValArray, 8);			//
	
	int max = getAbsMaxInt(rawValArray, 8);					// Off-Track Detection
	if (max<6)												//
	{														//	TODO:
		digitalWrite(LED_BUILTIN, HIGH);					//  Try to measure both levels
	}														//  induvidually
	else													//
	{														//
		digitalWrite(LED_BUILTIN, LOW);						//
	}							
	
	motorControl.setSpeed(10.0);

	motorControl.updateController(speedSens.getSpeed());	// Regler Updaten
	directionControl.updateController(average);				// 
	directionControl.updateDirection();						// 
	motorControl.updateMotor();								// 


	//Serial.println(directionControl.getDirection());		// Debug
	//Serial.println(average);								//
	//Serial.print("Speed: ");								//
	//Serial.println(speedSens.getSpeed());					//
	//Serial.println("\nSoll:");							//
	//Serial.println(30.0*reltime / 3000.0);				//
	
	//delay(15);
}

/**
 * The main speed interrupt handler
 */
void speedInterrupt()
{
	unsigned long newMicros = micros();
	speedSens.interrupt(newMicros - lastMicros);
	lastMicros = newMicros;
}

/**
 * Gibt den Durschnitt des Arrays zurück
 */
double getAverage(unsigned long *buff, int len)
{
	double avg = 0.0;
	for (int i = 0; i<len; i++)
	{
		avg += (double)buff[i] / len;
	}
	return avg;
}


int getAverageInt(int *buff, int len)
{
	long avg = 0;
	for (int i = 0; i < len; i++)
	{
		avg += buff[i];
	}
	return avg/len;
}

int getAbsMaxInt(int *buff, int len)
{
	int max = 0;
	for (int i = 0; i < len; i++)
	{
		if (max < buff[i] || buff[i] < ((-1)*max))
		{
			max = buff[i];
			if (max < 0)
			{
				max *= -1;
			}
		}
	}
	return max;
}


/**
 * The following code must not be changed
 */
void staticInitCode()
{
	pinMode(pin_deadManSwitch, INPUT_PULLUP);	//Set pullup for dead man switch
	pinMode(9, OUTPUT);
	digitalWrite(9, LOW);
}
