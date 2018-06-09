
#include <Servo.h>
#include <string.h>
//#include <string>
#include <math.h>
#include "DirectionControl.h"
#include "SpeedSensor.h"
#include "motorControl.h"

// pin declaration
// do NOT use DigitalOut 0 & 1 (reserved for Serial)
const int pin_servo = 5;			//pin for steering control 
const int pin_motor = 3;			//pin for motor control
const int pin_deadManSwitch =  8;	//dead man swich pin
const int pin_spuleLeft = 0;		//Left resonant circuit
const int pin_spuleRight = 0;		//Right resonant circuit
const int pin_speedSensor = 2;		//Input signal for speed sensor signal; Pin2: INT0


//init global variables and objects
const int directionServoCenter = 0;	//Center level for directionServo
const int directionMaxValue = 0;
const int directionMinValue = 0;
const int ofTrackDetectionLevel = 0;// minValue for off-track detection
DirectionControl directionControl = DirectionControl(
	pin_servo,																		// Servo-Pin
	{directionMaxValue,directionMinValue,directionServoCenter},						// directionParamSet
	{0, 0, 0, 0, directionServoCenter, 0, directionMinValue, directionMaxValue}		// pidParamSet
);

const double idleSpeed = 0;			//Speed setpoint for idle
MotorControl motorControl = MotorControl(pin_deadManSwitch, pin_motor, {2.502, 6.54118, 0, 0, 0, 0, 5 /*5*/, 253}); // okay: {5.0, 5.0, 0, 0, 0, 0, 5 /*5*/, 253} 
// double p, i, d, soll, base, antiWindup, outMin, outMax;

SpeedSensor speedSens = SpeedSensor(pin_speedSensor);

unsigned long lastMicros = 0;

Servo servo;


void setup()
{
	staticInitCode();	//this function call must not be removed
	//debugging
	Serial.begin(9600); // for serial logging
	pinMode(LED_BUILTIN, OUTPUT);  // declare onboard-led (indicates "on track")
	
	//directionControl.testServo();

	//servo.attach(pin_servo);
	pinMode(pin_servo, OUTPUT);

	motorControl.setup();
	speedSens.setup();

	//pinMode(14, INPUT);
	motorControl.setState(MOTOR_STATES::STOP);
}

/**
 * Main loop
 */
void loop()
{
	static unsigned long startMicros = micros();

	//motorControl.setDuty(6);

	//motorControl.updateController(speedSens.getSpeed());


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


	const long T = 2000000;
	int pos;
	long reltime = (micros() - startMicros) % T;
	if (reltime < T / 2)
	{
		pos = map(reltime, 0, T, 0, 180);
	}
	else
	{
		pos = map(T-reltime, 0, T, 0, 180);
		//servo.write(50.0* (T-reltime) / T - 25.0);
	}
	servo.write(pos);
	delay(15);

	//motorControl.setSpeed(20);

	//motorControl.updateMotor();

	//Serial.print("Speed: ");
	//Serial.println(speedSens.getSpeed());
	//Serial.println("\nSoll:");
	//Serial.println(30.0*reltime / 3000.0);
	//delay(10);
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

/**
 * The following code must not be changed
 */
void staticInitCode()
{
	pinMode(pin_deadManSwitch, INPUT_PULLUP);	//Set pullup for dead man switch
	pinMode(9, OUTPUT);
	digitalWrite(9, LOW);
}
