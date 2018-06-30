
#include "PID.h"
#include "Arduino.h"

PID::PID(pidParameterSet parameterIn)
{
	this->p = parameterIn.p;
	this->i = parameterIn.i;
	this->d = parameterIn.d;
	this->soll = parameterIn.soll;
	this->base = parameterIn.base;				// Zeittakt für PID, Zeitintervall zwischen Berechnungszeitpunkten
	this->antiWindup = parameterIn.antiWindup;	// Letzter Wert des Integrals, wenn die Stellgroeße das Maximum erreicht hat.
	this->outMin = parameterIn.outMin;			// Lower limit for output
	this->outMax = parameterIn.outMax;			// Upper limit for output
}

PID::~PID() {}

double PID::calcOutput(double input)
{
	double currentError = soll - input;

	unsigned long newMicros = micros();					// Get time since last call of this function
	double time_delta = (newMicros - lastMicros)/1e6;	//
	lastMicros = newMicros;								//

	antiWindup = integral;								// Save old integral value
	integral += time_delta * currentError;				// Update integral

	double output = base;
	output += p*currentError;							// Calculate new output
	output += i*integral;								//
	output += d*(currentError-lastError)/time_delta;	//

	if (output < outMin)					// Output has to be bounded
	{										// between outMin & outMax
		integral = antiWindup;				//
		output = outMin;					//
	}										//
	if(output > outMax)						//
	{										//
		integral = antiWindup;				//
		output = outMax;					//
	}										//

	lastError = currentError;				// Save currentError for differential calculation in next call
	
											//Serial.print("Error: ");
	//Serial.println(integral);
	return output;
}

/**
 * Returns the center output value of the PID
 * See PID.h for explanation
 */
double PID::getBase()	
{
	return base;
}

void PID::resetIntegrator()
{
	integral = 0;
}

void PID::setSoll(double soll)
{
	this->soll = soll;
}