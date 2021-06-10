#pragma once

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class RaspiI2C
{
public:
    void init(uint8_t address)
    {
        this->address_ = address;
        this->fd_ = wiringPiI2CSetup(this->address_);
        if(this->fd_ < 0)
        {
            printf("Error : please check you i2c address\n");
        }
    }
    void writeReg8(uint8_t reg, uint8_t value)
    {
        wiringPiI2CWriteReg8(fd_, reg, value);
    }
    void writeReg16(uint16_t reg, uint16_t value)
    {
        wiringPiI2CWriteReg16(fd_, reg, value);
    }
    void writeBit8(uint8_t value)
    {
        wiringPiI2CWrite(fd_, value);
    }
    uint8_t readReg8(uint8_t reg)
    {
        return wiringPiI2CReadReg8(fd_, reg);
    }
    uint16_t readReg16(uint16_t reg)
    {
        return wiringPiI2CReadReg16(fd_, reg);
    }
    uint8_t readBit8(uint8_t reg)
    {
        return wiringPiI2CRead(fd_);
    }
    int geti2cfd(){return fd_;}
    uint8_t getaddress(){return address_;}
private:
    int fd_;
    uint8_t address_;
};