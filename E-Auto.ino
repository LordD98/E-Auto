//
// Speed 3 & p=2,i=0,d=0 WORKS!
//
//	TODO: directionControl kleiner Fehler => schnell, großer Fehler => langsam
//

#include <Servo.h>
#include <string.h>
#include <math.h>
#include "DirectionControl.h"
#include "SpeedSensor.h"
#include "motorControl.h"

// pin declaration
// do NOT use DigitalOut 0 & 1 (reserved for Serial)
const int pin_servo = 6;			//pin for steering control 
const int pin_motor = 3;			//pin for motor control
const int pin_deadManSwitch =  8;	//dead man swich pin
const int pin_spuleLeft = A1;		//Left resonant circuit
const int pin_spuleRight = A0;		//Right reso nant circuit
const int pin_speedSensor = 2;		//Input signal for speed sensor signal; Pin2: INT0

//init global variables and objects
const int directionCenter = 0;
const int directionMaxValue = 400;
const int directionMinValue = -400;

const int directionServoCenter = 101;	//Center level for directionServo
const int directionServoMaxValue = 180;	// 101+70 = 171
const int directionServoMinValue = 0;	// actually 101-70 = 31
const int ofTrackDetectionLevel = 0;// minValue for off-track detection

DirectionControl directionControl = DirectionControl(
	pin_servo,																	// Servo-Pin
	{directionMaxValue,directionMinValue,directionCenter},						// directionParamSet
	{ 0.30, 0.0, 0.0, directionCenter, directionServoCenter, 0, directionServoMinValue, directionServoMaxValue }		// pidParamSet
	// p=0.3!
);

const double idleSpeed = 0;			//Speed setpoint for idle
#define LOW_SPEED 8.0	// SAFE: 8.0 & 20.0
#define HIGH_SPEED 23.0 //
MotorControl motorControl = MotorControl(pin_deadManSwitch, pin_motor, {2.0, 2.0, 0, LOW_SPEED, 0, 0, 5, 253});	// Works well
// These values are also okay: {5.0, 5.0, 0, 0, 0, 0, 5, 253} 

SpeedSensor speedSens = SpeedSensor(pin_speedSensor);

unsigned long lastMicros = 0;

#define FILTER_LEN 32
int rawValArray[FILTER_LEN];
int rawValArray_left[FILTER_LEN];
int rawValArray_right[FILTER_LEN];
int center_average;

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

	#define CALIB_LEN 32										// Calibration
	int calibArray[CALIB_LEN];									//
	int sensor_left;											//
	int sensor_right;											//
	for (int i = 0; i<CALIB_LEN; i++)							//
	{															//
		sensor_left = analogRead(pin_spuleLeft);				// Direction-Sensor
		sensor_right = analogRead(pin_spuleRight);				// Werte auslesen
		calibArray[i] = sensor_left - sensor_right;				//
		delay(1);											
	}
	center_average = getAverageInt(calibArray, CALIB_LEN);				//
	//directionControl.updateCenterPos(center_average);					//
	pinMode(A5, OUTPUT);
	pinMode(A3, OUTPUT);
	digitalWrite(A5, HIGH);
	digitalWrite(A3, LOW);

	motorControl.setSpeed(LOW_SPEED);
	motorControl.setState(MOTOR_STATES::RUN);	// Start motor
	//motorControl.softStart(10.0, &speedSens);
}

/**
 * Main loop
 */
void loop()
{
	static unsigned long startMicros = micros();
#define MULT 1
	//directionControl.set_p(static_cast<double>(analogRead(A4))/511.0);
	//Serial.println(static_cast<double>(analogRead(A4)) / 511.0);
	static byte index = 0;													// Direction-Sensor
	int sensor_left = MULT*analogRead(pin_spuleLeft);							// Werte auslesen
	int sensor_right = MULT*analogRead(pin_spuleRight);							//
	rawValArray[index] = (sensor_left-sensor_right);						//
	rawValArray_left[index] = sensor_left;									//
	rawValArray_right[index] = sensor_right;								//
	index = (index + 1)  % FILTER_LEN;										//
	int average = getAverageInt(rawValArray, FILTER_LEN) - center_average;	//
	int averageLeft = getAverageInt(rawValArray_left, FILTER_LEN);			//
	int averageRight = getAverageInt(rawValArray_right, FILTER_LEN);		//

	if (averageLeft < 15 && averageRight < 15)	// old values left<15 && right<15 new left<1 right<1
	{
		motorControl.setState(MOTOR_STATES::STOP);		// Off track
		digitalWrite(LED_BUILTIN, LOW);
	}
	else
	{
		motorControl.setState(MOTOR_STATES::RUN);
		digitalWrite(LED_BUILTIN, HIGH);
	}

	/*
	if (averageLeft < 20 && averageRight < 20)
	{
		motorControl.setState(MOTOR_STATES::STOP);		// Off track
		digitalWrite(LED_BUILTIN, LOW);
	}
	else
	{
		motorControl.setState(MOTOR_STATES::RUN);
		digitalWrite(LED_BUILTIN, HIGH);
		if (averageLeft < 35 && averageRight < 100)
		{
			// Unter Spule right
			if (averageLeft-averageRight > 0)
			{
				average = -200;
			}
		}
		else if (averageRight < 35 && averageLeft < 100)
		{
			// Unter Spule left
			if (averageLeft-averageRight < 0)
			{
				average = 200;
			}
		}
	
	}
	*/
	/*
	int max = getAbsMaxInt(rawValArray, FILTER_LEN);		// Off-Track Detection
	if (max<6)											//
	{														//	TODO:
		digitalWrite(LED_BUILTIN, HIGH);					//  Try to measure both levels
	}														//  induvidually
	else													//
	{														//
		digitalWrite(LED_BUILTIN, LOW);						//
	}							
	*/

	//Serial.print(sensor_left);
	//Serial.print(",");
	//Serial.print(averageLeft);
	//Serial.print(",");
	//Serial.print(sensor_right);
	//Serial.print(",");
	//Serial.print(averageRight);
	//Serial.print(",");
	//Serial.println(average);
	
	/*
	double new_Speed = static_cast<double>(average - center_average) / 200.0*10.0;
	//Serial.print((average - center_average));
	//Serial.print(",");
	//Serial.print(new_Speed);
	//Serial.print(",");
	if (new_Speed<0)
	{
		new_Speed *= -1;
	}
	new_Speed = 30 - new_Speed;
	motorControl.setSpeed(new_Speed);
	//Serial.print(new_Speed);
	//Serial.println("");
	*/

	//double diff = static_cast<double>(average);
	int max = getAbsMaxInt(rawValArray, FILTER_LEN);
	if (max > 60)	// old value 50 new 15 // test 60
	{
		motorControl.setSpeed(LOW_SPEED);
	}
	else
	{
		motorControl.setSpeed(HIGH_SPEED);	// old value 20 new 15
	}

	motorControl.updateController(speedSens.getSpeed());			// Regler Updaten
	//motorControl.setDuty(60);
	directionControl.updateController(average);	// old *1 new *10
	motorControl.updateMotor();										// 
	directionControl.updateDirection();								// 


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
