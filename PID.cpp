
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

	double output = 0;

	unsigned long newMicros = micros();		// Get time since last call of this function
	base = (newMicros - lastMicros)/1e6;	// and put in into base
	lastMicros = newMicros;					//

	antiWindup = integral;					// Save old integral value
	integral += base * currentError;		// Update integral

	output += p*currentError;					// Calculate new output
	output += i*integral;						//
	output += d*(currentError-lastError)/base;	//

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
	//Serial.println(currentError);
	return output;
}

/**
 * Return time intervall between two calls of calcOutput()
 * Use: ???
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