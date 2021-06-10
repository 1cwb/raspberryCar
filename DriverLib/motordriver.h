#pragma once
#include <iostream>
#include <functional>
#include <atomic>
#include <wiringPi.h>
#include <softPwm.h>
#include <softTone.h>
#include "pca9685.h"

#define BEEP_GPIO 2
#define IRCONTROL 7

//encoder from raspberry pi
#define DCMOTOR1_ENCODER_1   21
#define DCMOTOR1_ENCODER_2   22
#define DCMOTOR2_ENCODER_1   23
#define DCMOTOR2_ENCODER_2   24
#define DCMOTOR3_ENCODER_1   28
#define DCMOTOR3_ENCODER_2   29
#define DCMOTOR4_ENCODER_1   25
#define DCMOTOR4_ENCODER_2   27
//DC motor pwm gpio from pca9685
#define DCMOTOR1_PWM1        13
#define DCMOTOR1_PWM2        11
#define DCMOTOR2_PWM1        8
#define DCMOTOR2_PWM2        10
#define DCMOTOR3_PWM1        4
#define DCMOTOR3_PWM2        2
#define DCMOTOR4_PWM1        5
#define DCMOTOR4_PWM2        7
//servo pwm gpio from pca9685
#define SERVO_PWM1           0
#define SERVO_PWM2           1
#define SERVO_PWM3           14
#define SERVO_PWM4           15
#define SERVO_PWM5           9
#define SERVO_PWM6           12
#define SERVO_PWM7           3
#define SERVO_PWM8           6

typedef void(*EncoderCallBack)();
typedef void(*IRControlCallBack)(uint8_t key, bool continuePress);
// FOR IR
typedef enum
{
    E_IRVALUE_POWER = 0X00,
    E_IRVALUE_UP,
    E_IRVALUE_LED,
    E_IRVALUE_LEFT = 0X04,
    E_IRVALUE_VOICE,
    E_IRVALUE_RIGHT,
    E_IRVALUE_BACK = 0X08,
    E_IRVALUE_DOWN,
    E_IRVALUE_NEXT,
    E_IRVALUE_ADD = 0X0C,
    E_IRVALUE_NUM0,
    E_IRVALUE_SUB,
    E_IRVALUE_NUM1 = 0X10,
    E_IRVALUE_NUM2,
    E_IRVALUE_NUM3,
    E_IRVALUE_NUM4 = 0X14,
    E_IRVALUE_NUM5,
    E_IRVALUE_NUM6,
    E_IRVALUE_NUM7 = 0X18,
    E_IRVALUE_NUM8,
    E_IRVALUE_NUM9,
    E_IRVALUE_MAX
}E_IRVALUE;

typedef enum
{
    E_DCFORWARD = 0,
    E_DCBACKWORD,
    E_DCTURNLEFT,
    E_DCTURNRIGHT,
    E_DCSTOP,
    E_DCBREAK
}E_DCMOTOR_CMD;
typedef enum
{
    E_DCMOTORNUM_1 = 0,
    E_DCMOTORNUM_2,
    E_DCMOTORNUM_3,
    E_DCMOTORNUM_4,
    E_DCMOTORNUM_MAX
}EDCMOTORNUM;

typedef enum
{
    E_SERVONUM_1 = 0,
    E_SERVONUM_2,
    E_SERVONUM_3,
    E_SERVONUM_4,
    E_SERVONUM_5,
    E_SERVONUM_6,
    E_SERVONUM_7,
    E_SERVONUM_8,
    E_SERVONUM_MAX
}ESERVONUM;
class MotorDriverBoard;
class DcMotor
{
public:
    DcMotor();
    void run(E_DCMOTOR_CMD cmd);
    void setSpeed(uint8_t speed);
    inline uint8_t getSpeed() {return speed_;}
    inline E_DCMOTOR_CMD getmcmd() {return mcmd_;}
    inline void setMotorNum(EDCMOTORNUM motornum) {motornum_ = motornum;}
    inline EDCMOTORNUM getMotorNum() {return motornum_;}
    inline void setIn1Pin(uint8_t in1pin) { in1pin_ = in1pin;}
    inline uint8_t getIn1Pin() {return in1pin_;}
    inline void setIn2Pin(uint8_t in2pin) {in2pin_ = in2pin;}
    inline uint8_t getIn2Pin() {return in2pin_;}

    inline void setOut1Pin(uint8_t out1pin) {out1pin_ = out1pin;}
    inline uint8_t getOut1Pin() {return out1pin_;}
    inline void setOut2Pin(uint8_t out2pin) {out2pin_ = out2pin;}
    inline uint8_t getOut2Pin() {return out2pin_;}

    inline void setMotorDriverBoard(MotorDriverBoard* drvboard) {drvboard_ = drvboard;}
    inline MotorDriverBoard* getMotorDriverBoard() {return drvboard_;}

    inline int64_t getencount() {return encount_[motornum_];}
    inline static void increaseCount(EDCMOTORNUM n) {encount_[n] ++;}
    inline static void decreaseCount(EDCMOTORNUM n) {encount_[n] --;}

    inline void enableEncoder()
    {
        pinMode (out1pin_, INPUT);
        pinMode (out2pin_, INPUT);
        wiringPiISR (out1pin_, INT_EDGE_RISING, callback_);
    }
    inline void disableEncoder() {wiringPiISR (out1pin_, INT_EDGE_SETUP, [](){});}
    inline void setEncoderCallBack(EncoderCallBack callback) { callback_ = callback;}
private:
    uint8_t in1pin_;
    uint8_t in2pin_;
    uint8_t out1pin_;
    uint8_t out2pin_;
    uint8_t speed_;
    
    E_DCMOTOR_CMD mcmd_;
    EDCMOTORNUM motornum_;
    MotorDriverBoard* drvboard_;
    EncoderCallBack callback_;
    static std::atomic<int64_t> encount_[E_DCMOTORNUM_MAX];
};

class Servo
{
public:
    Servo();
    void setServoPulse(double pulse);
    void writeServo(uint8_t angle);
    inline uint8_t readDegrees() {return currentAngel_;}
    inline void setPwmPin(uint8_t pwmpin) {pwmpin_ = pwmpin;}
    inline uint8_t getPwmPin() {return pwmpin_;}
    inline void setServoNum(ESERVONUM servonum) {servonum_ = servonum;}
    inline uint8_t getServoNum() {return servonum_;}
    inline void setMotorDriverBoard(MotorDriverBoard* drvboard) {drvboard_ = drvboard;}
    inline MotorDriverBoard* getMotorDriverBoard() {return drvboard_;}
    inline void setPwmFreq(uint16_t pwmfreq) {pwmfreq_ = pwmfreq;}
    inline uint16_t getPwmFreq() {return pwmfreq_;}
private:
    uint8_t pwmpin_;
    uint16_t pwmfreq_;
    uint8_t currentAngel_;
    ESERVONUM servonum_;
    MotorDriverBoard* drvboard_;
};

class MotorDriverBoard
{
public:
    MotorDriverBoard(uint8_t addr = 0x60);
    void init(uint16_t freq = 50);

    void setPWM(uint8_t pin, uint16_t val);
    void setPin(uint8_t pin, uint8_t val);//set pin keep high or low

    DcMotor* getMotor(EDCMOTORNUM n);
    Servo* getServo(ESERVONUM n);
    void beepInit()
    {
        softToneCreate(BEEP_GPIO);//init beep gpio
    }
    void beepRun(uint32_t beepfreq = 2000);
    void beepStop();

    void irInit(IRControlCallBack cb);
private:
    uint8_t addr_;
    uint16_t freq_;
    DcMotor dcmotors_[E_DCMOTORNUM_MAX];
    Servo   servos[E_SERVONUM_MAX];
    Pca9685 pcadevice_; 
};


