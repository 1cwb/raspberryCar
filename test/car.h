#pragma once

#include "motordriver.h"
struct irdata
{
    irdata(E_IRVALUE k, bool c)
    {
        key = k;
        isContinue = c;
    }
    irdata():key(E_IRVALUE_MAX), isContinue(false){}
    E_IRVALUE key;
    bool isContinue;
};
class Car 
{
public:
    Car();
    ~Car();
    void forward();
    void back();
    void turnleft();
    void turnright();
    void stop();
    void setSpeed(uint8_t speed);
    uint8_t getSpeed(){return dc1->getSpeed();}
    bool isRun(){return brun;}
    void beepRun(uint16_t freq = 5000){drvboad.beepRun(freq);}
    void beepStop(){drvboad.beepStop();}
private:
    MotorDriverBoard drvboad;
    DcMotor* dc1;
    DcMotor* dc2;
    DcMotor* dc3;
    DcMotor* dc4;
    bool brun;
};