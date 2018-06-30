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
	/*
	 * Base is the center output value of the PID
	 * Example: Direction Control
	 * If no integral part is used and error is 0, the car should go straight forward
	 * and the servo should be set to 101 (=base).
	 **/

	unsigned long lastMicros = 0;
    double antiWindup;

};