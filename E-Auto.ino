
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

const int pin_test = 19;


//init global variables and objects
const int directionCenter = -150;
const int directionMaxValue = 400;
const int directionMinValue = -400;

const int directionServoCenter = 101;	//Center level for directionServo
const int directionServoMaxValue = 171;	// 101+70 = 171
const int directionServoMinValue = 9;	// actually 101-70 = 31
const int ofTrackDetectionLevel = 0;// minValue for off-track detection
DirectionControl directionControl = DirectionControl(
	pin_servo,																		// Servo-Pin
	{directionMaxValue,directionMinValue,directionCenter},						// directionParamSet
	{ (-1.0)* 70.0/400.0 * 0.5, 0, 0, directionCenter, directionServoCenter, 0, directionServoMinValue, directionServoMaxValue}		// pidParamSet
);

const double idleSpeed = 0;			//Speed setpoint for idle
MotorControl motorControl = MotorControl(pin_deadManSwitch, pin_motor, {2.502, 6.54118, 0, 0, 0, 0, 5, 253});	// Works well
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
	
	//pinMode(pin_test, INPUT);

	directionControl.setup();
	directionControl.testServo();

	//while (1);

	//pinMode(12, OUTPUT);		// Just for Voltage divider
	//digitalWrite(12, HIGH);		// Just for Voltage divider


	motorControl.setup();
	speedSens.setup();

	//pinMode(14, INPUT);
	motorControl.setState(MOTOR_STATES::STOP);	// Do start Motor
	motorControl.softStart(20.0, &speedSens);
}

/**
 * Main loop
 */
void loop()
{
	static unsigned long startMicros = micros();

	//motorControl.setDuty(6);

	motorControl.updateController(speedSens.getSpeed());

	static byte index = 0;
							// discard the value as it must be a glitch

	int sensor_left = analogRead(pin_spuleLeft);
	int sensor_right = analogRead(pin_spuleRight);

	rawValArray[index] = sensor_left-sensor_right;
	index = (index + 1)  % 8;
	int average = getAverageInt(rawValArray, 8);
	int max = getAbsMaxInt(rawValArray, 8);
	if (max<6)
	{
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else
	{
		digitalWrite(LED_BUILTIN, LOW);
	}

	directionControl.updateController(average);
	directionControl.updateDirection();
	//Serial.println(average);  



	motorControl.setSpeed(20);
	motorControl.updateMotor();

	//Serial.print("Speed: ");
	//Serial.println(speedSens.getSpeed());
	//Serial.println("\nSoll:");
	//Serial.println(30.0*reltime / 3000.0);
	//delay(10);
	
	delay(15);
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
 * Gibt den Durschnitt des Arrays zur�ck
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
