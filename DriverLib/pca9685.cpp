#include "pca9685.h"
#include <cassert>

Pca9685::Pca9685(uint8_t addr)
{
    i2caddr_ = addr;
}
void Pca9685::init(void)
{
    RaspiI2C::init(i2caddr_);
    assert(RaspiI2C::geti2cfd() > 0);
    //printf("fd is %d\n",RaspiI2C::geti2cfd());
    reset();
}
void Pca9685::reset(void)
{
    write8(PCA9685_MODE1, 0X00);
}
void Pca9685::setPWMFreq(float freq)
{
    freq *= 0.9;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;

    uint8_t prescale = floor(prescaleval + 0.5);

    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7f) | 0x10; //sleep
    write8(PCA9685_MODE1, newmode); // go to sleep
    write8(PCA9685_PRESCALE, prescale); //set prescaler
    write8(PCA9685_MODE1, oldmode);
    delay(5);
    write8(PCA9685_MODE1, oldmode | 0xa1);
}
void Pca9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    write8(LED0_ON_L + 4*num, on & 0xff);
    write8(LED0_ON_H + 4*num, on >> 8);
    write8(LED0_OFF_L + 4*num, off & 0xff);
    write8(LED0_OFF_H + 4*num, off >> 8);
}

uint8_t Pca9685::read8(uint8_t addr)
{
    return readReg8(addr);
}
void Pca9685::write8(uint8_t addr, uint8_t d)
{
    writeReg8(addr, d);
}