
#include "PID.h"
#include "Arduino.h"

PID::PID(pidParameterSet parameterIn)
{
	this->p = parameterIn.p;
	this->i = parameterIn.i;
	this->d = parameterIn.d;
	this->soll = parameterIn.soll;
	this->base = parameterIn.base;				// 
	this->antiWindup = parameterIn.antiWindup;	// Letzter Wert des Integrals, wenn die Stellgroeße das Maximum erreicht hat.
	this->outMin = parameterIn.outMin;
	this->outMax = parameterIn.outMax;
	// Zeittakt für PID, Intervall zwischen Berechnungszeitpunkten ???
}

PID::~PID() {}

double PID::calcOutput(double input)
{
	double currentError = soll - input;

	double output = 0;

	if (input < 0.01)
	{
		integral = 0;
	}

	unsigned long newMicros = micros();
	base = (newMicros - lastMicros)/1e6;
	lastMicros = newMicros;


	antiWindup = integral;
	integral += base * currentError;

	output += p*currentError;
	output += i*integral;
	output += d*(currentError-lastError)/base;

	if (output < outMin)
	{
		integral = antiWindup;
		output = outMin;
	}
	if(output > outMax)
	{
		integral = antiWindup;
		output = outMax;
	}

	lastError = currentError;
	return output;
}

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