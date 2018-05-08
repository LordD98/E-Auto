#pragma once

extern void speedInterrupt();

class SpeedSensor {
  public:
    SpeedSensor(int);
    ~SpeedSensor();

    void interrupt(unsigned long microseconds);
    bool getState();
    double getSpeed();
    void setup();
    
  private:
    int speedSendPin;
    
  
    bool state = false;    
    long rawValArray[8] = {0};
    int arraySize = 8;
    unsigned long lastTurn = 0;
    unsigned long avgPos = 0;
    unsigned long avgVal = 0;
    unsigned int initOk = 0;
    double currentSpeed = 0;
};

