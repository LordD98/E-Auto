#include "PID.h"

PID::PID(pidParameterSet parameterIn)
{
	PID::p = parameterIn.p;
	PID::i = parameterIn.i;
	PID::d = parameterIn.d;
	PID::soll = parameterIn.soll;
	PID::base = parameterIn.base;
	PID::antiWindup = parameterIn.antiWindup;
	PID::outMin = parameterIn.outMin;
	PID::outMax = parameterIn.outMax;
}

PID::~PID() {}

double PID::calcOutput(double input)
{
	return 0;
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
	PID::soll = soll;
}

