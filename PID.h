#pragma once

struct pidParameterSet
{
  double p, i, d, soll, base, antiWindup, outMin, outMax;
};


class PID
{
  public:
    PID(pidParameterSet);
    ~PID();

    void setSoll(double soll);
    void resetIntegrator();
    double calcOutput(double input);
    double getBase();
        
  private:   
    double lastError, integral;
    double p, i, d;
	double outMin = 0, outMax = 255, soll, base;

	unsigned long lastMicros = 0;
    double antiWindup;

};