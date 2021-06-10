#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include "car.h"
#include "message.h"
using namespace std;
extern SafeQueue<irdata> irevent;

void txx()
{
    while(1)
    {

    }
}
int main()
{
    Car ca;
    irdata data;
    E_IRVALUE irv;
    uint8_t tempspeed = 0;
    uint8_t speed = 120;
    while(true)
    {
        if(irevent.popWait(&data,5000))
        {
            printf("key is %x, contune is %s\n",data.key,data.isContinue ? "true": "false");
            switch(data.key)
            {
                case E_IRVALUE_POWER:
                    if(ca.isRun())
                    {
                        ca.stop();
                    }
                    ca.beepStop();
                    break;
                case E_IRVALUE_UP:
                    if(irv == data.key && data.isContinue)
                    {
                        if(tempspeed < 255 - 10)
                        {
                            tempspeed += 10;
                            if(tempspeed >= 255 - 10)
                            {
                                tempspeed = 255;
                            }
                            ca.setSpeed(tempspeed);
                            printf("speed is %u\n",tempspeed);
                        }
                    }
                    else
                    {
                        ca.setSpeed(speed);
                        tempspeed = speed;
                    }
                    ca.forward();
                    irv = data.key;
                    break;
                    case E_IRVALUE_DOWN:
                    if(irv == data.key && data.isContinue)
                    {
                        if(tempspeed < 255 - 10)
                        {
                            tempspeed += 10;
                            if(tempspeed >= 255 - 10)
                            {
                                tempspeed = 255;
                            }
                            ca.setSpeed(tempspeed);
                            printf("speed is %u\n",tempspeed);
                        }
                    }
                    else
                    {
                        ca.setSpeed(speed);
                        tempspeed = speed;
                    }
                    ca.back();
                    irv = data.key;
                    break;
                    case E_IRVALUE_LEFT:
                    if(irv == data.key && data.isContinue)
                    {
                        if(tempspeed < 255 - 10)
                        {
                            tempspeed += 10;
                            if(tempspeed >= 255 - 10)
                            {
                                tempspeed = 255;
                            }
                            ca.setSpeed(tempspeed);
                            printf("speed is %u\n",tempspeed);
                        }
                    }
                    else
                    {
                        ca.setSpeed(speed);
                        tempspeed = speed;
                    }
                    ca.turnleft();
                    irv = data.key;
                    break;
                    case E_IRVALUE_RIGHT:
                    if(irv == data.key && data.isContinue)
                    {
                        if(tempspeed < 255 - 10)
                        {
                            tempspeed += 10;
                            if(tempspeed >= 255 - 10)
                            {
                                tempspeed = 255;
                            }
                            ca.setSpeed(tempspeed);
                            printf("speed is %u\n",tempspeed);
                        }
                    }
                    else
                    {
                        ca.setSpeed(speed);
                        tempspeed = speed;
                    }
                    ca.turnright();
                    irv = data.key;
                    break;
                    case E_IRVALUE_LED:
                    ca.beepRun();
                        
                        break;
                default:
                    break;
            }
        }
    }
    return 0;
}