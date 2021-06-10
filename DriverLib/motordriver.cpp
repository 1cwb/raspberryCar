#include "motordriver.h"
#include <cstring>
using namespace std;
std::atomic<int64_t> DcMotor::encount_[E_DCMOTORNUM_MAX];
DcMotor::DcMotor()
{
    drvboard_ = NULL;
    motornum_ = E_DCMOTORNUM_MAX;
    speed_ = in1pin_ = in2pin_ = out1pin_ = out2pin_ =0;
    memset(encount_, 0, sizeof(encount_));
}
void DcMotor::run(E_DCMOTOR_CMD cmd)
{
    mcmd_ = cmd;
    switch(cmd)
    {
        case E_DCFORWARD:
            drvboard_->setPin(in2pin_, LOW);
            drvboard_->setPWM(in1pin_,speed_ * 16);
            break;
        case E_DCBACKWORD:
            drvboard_->setPin(in1pin_, LOW);
            drvboard_->setPWM(in2pin_,speed_ * 16);
            break;
        case E_DCSTOP:
            drvboard_->setPin(in1pin_, LOW);
            drvboard_->setPin(in2pin_, LOW);
            break;
        case E_DCBREAK:
            drvboard_->setPin(in1pin_, HIGH);
            drvboard_->setPin(in2pin_, HIGH);
            break;
        default: break;
    }
}
void DcMotor::setSpeed(uint8_t speed)
{
    speed_ = speed;
    run(mcmd_);
}


Servo::Servo()
{
    drvboard_ = NULL;
    servonum_ = E_SERVONUM_MAX;
    pwmfreq_ = 0;
    pwmpin_ = 0;
    currentAngel_ = 0;
}
void Servo::setServoPulse(double pulse)
{
    double pulseLength = 1000000; //us
    pulseLength /= 50;
    pulseLength /= 4096;
    pulse *= 1000;
    pulse /= pulseLength;
    drvboard_->setPWM(pwmpin_, pulse);
}
void Servo::writeServo(uint8_t angle)
{
    if(angle > 170)
    {
        angle = 170;
    }
    double pulse;
    pulse = 0.5 + angle / 90.0;
    setServoPulse(pulse);
    currentAngel_ = angle;
}

MotorDriverBoard::MotorDriverBoard(uint8_t addr) : addr_(addr), pcadevice_(addr)
{
    wiringPiSetup(); 
}
void MotorDriverBoard::init(uint16_t freq)
{
    pcadevice_.init();
    freq_ = freq;
    pcadevice_.setPWMFreq(freq);
    for(uint8_t i = 0; i < 16; i++)
    {
        pcadevice_.setPWM(i, 0, 0);
    }
}

void MotorDriverBoard::setPWM(uint8_t pin, uint16_t val)
{
    if(val > 4095)
    {
        pcadevice_.setPWM(pin, 4096, 0);//always off
    }
    else
    {
        pcadevice_.setPWM(pin, 0, val);//after val it off
    }
}
void MotorDriverBoard::setPin(uint8_t pin, uint8_t val)
{
    if(val == 0)
    {
        pcadevice_.setPWM(pin, 0, 0); //always on
    }
    else
    {
        pcadevice_.setPWM(pin, 4096, 0); //always off
    }
}

DcMotor* MotorDriverBoard::getMotor(EDCMOTORNUM n)
{
    if(n > E_DCMOTORNUM_MAX)
    {
        return NULL;
    }
    if(dcmotors_[n].getMotorNum() == E_DCMOTORNUM_MAX)
    {
        dcmotors_[n].setMotorNum(n);
        dcmotors_[n].setMotorDriverBoard(this);
        switch(n)
        {
            case E_DCMOTORNUM_1:
                dcmotors_[n].setIn1Pin(DCMOTOR1_PWM1);
                dcmotors_[n].setIn2Pin(DCMOTOR1_PWM2);
                dcmotors_[n].setOut1Pin(DCMOTOR1_ENCODER_1);
                dcmotors_[n].setOut2Pin(DCMOTOR1_ENCODER_2);
                dcmotors_[n].setEncoderCallBack([](){
                    if(digitalRead (DCMOTOR1_ENCODER_1) == HIGH) //REGISTER ENCORDER CALLBACK
                    { 
                       if(digitalRead(DCMOTOR1_ENCODER_2) == HIGH)
                        {
                            DcMotor::increaseCount(E_DCMOTORNUM_1);
                        }
                        else
                        {
                            DcMotor::decreaseCount(E_DCMOTORNUM_1);
                        }
                    }
                });
                break;
            case E_DCMOTORNUM_2:
                dcmotors_[n].setIn1Pin(DCMOTOR2_PWM1);
                dcmotors_[n].setIn2Pin(DCMOTOR2_PWM2);
                dcmotors_[n].setOut1Pin(DCMOTOR2_ENCODER_1);
                dcmotors_[n].setOut2Pin(DCMOTOR2_ENCODER_2);
                dcmotors_[n].setEncoderCallBack([](){
                    if(digitalRead (DCMOTOR2_ENCODER_1) == HIGH)
                    { 
                       if(digitalRead(DCMOTOR2_ENCODER_2) == HIGH)
                        {
                            DcMotor::increaseCount(E_DCMOTORNUM_2);
                        }
                        else
                        {
                            DcMotor::decreaseCount(E_DCMOTORNUM_2);
                        }
                    }
                });
                break;
            case E_DCMOTORNUM_3:
                dcmotors_[n].setIn1Pin(DCMOTOR3_PWM1);
                dcmotors_[n].setIn2Pin(DCMOTOR3_PWM2);
                dcmotors_[n].setOut1Pin(DCMOTOR3_ENCODER_1);
                dcmotors_[n].setOut2Pin(DCMOTOR3_ENCODER_2);
                dcmotors_[n].setEncoderCallBack([](){
                    if(digitalRead (DCMOTOR3_ENCODER_1) == HIGH)
                    { 
                       if(digitalRead(DCMOTOR3_ENCODER_2) == HIGH)
                        {
                            DcMotor::increaseCount(E_DCMOTORNUM_3);
                        }
                        else
                        {
                            DcMotor::decreaseCount(E_DCMOTORNUM_3);
                        }
                    }
                });
                break;
            case E_DCMOTORNUM_4:
                dcmotors_[n].setIn1Pin(DCMOTOR4_PWM1);
                dcmotors_[n].setIn2Pin(DCMOTOR4_PWM2);
                dcmotors_[n].setOut1Pin(DCMOTOR4_ENCODER_1);
                dcmotors_[n].setOut2Pin(DCMOTOR4_ENCODER_2);
                dcmotors_[n].setEncoderCallBack([](){
                    if(digitalRead (DCMOTOR4_ENCODER_1) == HIGH)
                    { 
                       if(digitalRead(DCMOTOR4_ENCODER_2) == HIGH)
                        {
                            DcMotor::increaseCount(E_DCMOTORNUM_4);
                        }
                        else
                        {
                            DcMotor::decreaseCount(E_DCMOTORNUM_4);
                        }
                    }
                });
                break;
            default:
                break;
        }
    }
    return &dcmotors_[n];
}

Servo* MotorDriverBoard::getServo(ESERVONUM n)
{
    if(n >= E_SERVONUM_MAX)
    {
        return NULL;
    }
    if(servos[n].getServoNum() == E_SERVONUM_MAX)
    {
        switch (n)
        {
            case E_SERVONUM_1:
                servos[n].setPwmPin(SERVO_PWM1);
                break;
            case E_SERVONUM_2:
                servos[n].setPwmPin(SERVO_PWM2);
                break;
            case E_SERVONUM_3:
                servos[n].setPwmPin(SERVO_PWM3);
                break;
            case E_SERVONUM_4:
                servos[n].setPwmPin(SERVO_PWM4);
                break;
            case E_SERVONUM_5:
                servos[n].setPwmPin(SERVO_PWM5);
                break;
            case E_SERVONUM_6:
                servos[n].setPwmPin(SERVO_PWM6);
                break;
            case E_SERVONUM_7:
                servos[n].setPwmPin(SERVO_PWM7);
                break;
            case E_SERVONUM_8:
                servos[n].setPwmPin(SERVO_PWM8);
                break;
            default:
                break;
        }
        servos[n].setMotorDriverBoard(this);
        servos[n].setServoNum(n);
        servos[n].setPwmFreq(freq_);
    }
    return &servos[n];
}

void MotorDriverBoard::beepRun(uint32_t beepfreq)
{
    softToneWrite (BEEP_GPIO, beepfreq);
}
void MotorDriverBoard::beepStop()
{
    softToneWrite (BEEP_GPIO, 0);
}

void MotorDriverBoard::irInit(IRControlCallBack cb)
{
    static uint64_t oldtime = 0;
    static uint64_t newtime = 0;
    static int ircount = 0;
    static char irval = 0;
    static char oldval = 0;
    static char nirval = 0;
    static bool hasgethead = false;
    static bool isfirstTimeUse = true;
    static IRControlCallBack ircallback_ = NULL;
    ircallback_ = cb;
    pinMode (IRCONTROL, INPUT);
    pullUpDnControl(IRCONTROL, PUD_UP);
    wiringPiISR (IRCONTROL, INT_EDGE_FALLING, [](){
        newtime = micros();
        if(oldtime == 0)
        {
            oldtime = newtime;
            return;
        }
        uint64_t temptime = newtime - oldtime ;
        //printf("tempTime is %llu\n",temptime);
        if(temptime <= 14000LLU && temptime >= 1000LLU)
        {
            if(temptime >= 13500)
            {
                hasgethead = true;
            }
            else if(hasgethead)
            {
                if(temptime < 1600)
                
                {
                    if(ircount >= 16 && ircount < 24)
                    {
                        irval &=  ~(1 << (ircount - 16));
                    }
                    if(ircount >= 24 && ircount < 32)
                    {
                        nirval &=  ~(1 << (ircount - 24));
                    }
                }
                else
                {
                    if(ircount >= 16 && ircount < 24)
                    {
                        irval |=  1 << (ircount - 16);
                    }
                    if(ircount >= 24 && ircount < 32)
                    {
                        nirval |=  1 << (ircount - 24);
                    }
                }
                ircount ++;
                if(ircount >= 32)
                {
                    oldval = irval;
                    if(ircallback_)
                    {
                        ircallback_(irval, false);
                    }
                    irval = 0;
                    nirval = 0;
                    oldtime = 0;
                    ircount = 0;
                    hasgethead = false;
                    isfirstTimeUse = false;
                }
            }
        }
        else if(temptime >= 96000 && temptime <= 97000 && !isfirstTimeUse)
        {
            ircallback_(oldval, true);
        }
        else
        {
            oldtime = 0;
            ircount = 0;
            irval = 0;
            nirval = 0;
            hasgethead = false;
        }
        oldtime = newtime;
    });
}
