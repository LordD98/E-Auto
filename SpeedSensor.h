#pragma once

#include <Arduino.h>
#include <limits.h>

extern void speedInterrupt();
extern unsigned long lastMicros;

class SpeedSensor
{
  public:
    SpeedSensor(int);
    ~SpeedSensor();

    void interrupt(unsigned long microseconds);
    bool getState();
    double getSpeed();
    void setup();

	static unsigned long MIN_INT;
    
  private:
    int speedSendPin;
    
  
    bool state = false;    
	unsigned long rawValArray[8] = { ULONG_MAX, ULONG_MAX, ULONG_MAX, ULONG_MAX, ULONG_MAX, ULONG_MAX, ULONG_MAX, ULONG_MAX };
    int arraySize = 8;
    unsigned long lastTurn = 0;
    unsigned long avgPos = 0;
    unsigned long avgVal = 0;
    unsigned int initOk = 0;
    double currentSpeed = 0;	// in u/s (Umdrehung/sec)
};