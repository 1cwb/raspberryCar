#include "car.h"
#include "message.h"
SafeQueue<irdata> irevent(1024);
Car::Car():brun(false)
{
    drvboad.init(50);
    drvboad.beepInit();
    drvboad.irInit([](uint8_t key, bool continuePress){
        irdata kl((E_IRVALUE)key, continuePress);
        irevent.push(kl);
    });
    dc1 = drvboad.getMotor(E_DCMOTORNUM_1);
    dc2 = drvboad.getMotor(E_DCMOTORNUM_2);
    dc3 = drvboad.getMotor(E_DCMOTORNUM_3);
    dc4 = drvboad.getMotor(E_DCMOTORNUM_4);

    dc1->enableEncoder();
    dc2->enableEncoder();
    dc3->enableEncoder();
    dc4->enableEncoder();


    dc1->run(E_DCSTOP);
    dc2->run(E_DCSTOP);
    dc3->run(E_DCSTOP);
    dc4->run(E_DCSTOP);
}
Car::~Car()
{
    dc1->disableEncoder();
    dc2->disableEncoder();
    dc3->disableEncoder();
    dc4->disableEncoder();
    dc1->run(E_DCSTOP);
    dc2->run(E_DCSTOP);
    dc3->run(E_DCSTOP);
    dc4->run(E_DCSTOP);
    brun = false;
}
void Car::forward()
{
    dc1->run(E_DCFORWARD);
    dc2->run(E_DCFORWARD);
    dc3->run(E_DCFORWARD);
    dc4->run(E_DCFORWARD);
    brun = true;
}
void Car::back()
{
    dc1->run(E_DCBACKWORD);
    dc2->run(E_DCBACKWORD);
    dc3->run(E_DCBACKWORD);
    dc4->run(E_DCBACKWORD);
    brun = true;
}
void Car::turnleft()
{
    dc1->run(E_DCFORWARD);
    dc2->run(E_DCBACKWORD);
    dc3->run(E_DCBACKWORD);
    dc4->run(E_DCFORWARD);
    brun = true;
}
void Car::turnright()
{
    dc1->run(E_DCBACKWORD);
    dc2->run(E_DCBACKWORD);
    dc3->run(E_DCBACKWORD);
    dc4->run(E_DCBACKWORD);
    brun = true;
}
void Car::stop()
{
    dc1->run(E_DCSTOP);
    dc2->run(E_DCSTOP);
    dc3->run(E_DCSTOP);
    dc4->run(E_DCSTOP);
    brun = false;
}
void Car::setSpeed(uint8_t speed)
{
    dc1->setSpeed(speed);
    dc2->setSpeed(speed);
    dc3->setSpeed(speed);
    dc4->setSpeed(speed);
}


//MotorDriverBoard drvboad;