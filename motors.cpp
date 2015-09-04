//
// Created by Bryce Carter on 8/25/15.
//

#include "rMotors.h"


namespace RVR
{

    // ==============================================================
    // MotorProperties Class Member functions
    // ==============================================================

    RVR::MotorProperties::MotorProperties(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep,
                                          int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense)
    {
        this->IN1 = in1;
        this->IN2 = in2;
        this->I0 = i0;
        this->I1 = i1;
        this->I2 = i2;
        this->I3 = i3;
        this->I4 = i4;
        this->FAULT = fault;
        this->SLEEP = sleep;
        this->RESET = reset;
        this->DECAY = decay;
        this->POWER_RAIL = powerRail;
        this->V_REF = vRef;
        this->R_SENSE = rSense;
        this->I_FULL_SCALE = this->V_REF / (5 * this->R_SENSE) * 1000;
    }

    // ==============================================================
    // Motor Class Member functions
    // ==============================================================

    RVR::Motor::Motor(RVR::MotorName motorName)
    {
        // Assign the correct pin mapping for the motor
        switch (motorName)
        {
            case RVR::DRIVE_MOTOR_1:
                this->motorProperties = RVR::Motor::drive1MotorMapping;
                break;
            case RVR::DRIVE_MOTOR_2:
                this->motorProperties = RVR::Motor::drive2MotorMapping;
                break;
            case RVR::TREAT_MOTOR:
                this->motorProperties = RVR::Motor::treatMotorMapping;
                break;
            case RVR::CAMERA_MOTOR:
                this->motorProperties = RVR::Motor::cameraMotorMapping;
                break;
        }
        this->setupPins();
    }

    RVR::PowerRail* motorRail = RVR::PowerManager::getRail(RVR::RAIL12V0);
    const RVR::MotorProperties * const RVR::Motor::drive1MotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);
    const RVR::MotorProperties * const RVR::Motor::drive2MotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);
    const RVR::MotorProperties * const RVR::Motor::treatMotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);
    const RVR::MotorProperties * const RVR::Motor::cameraMotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);

    int Motor::setCurrentLimit(int currentLimit) {
        if (currentLimit > this->motorProperties->I_FULL_SCALE)
        {
            currentLimit = this->motorProperties->I_FULL_SCALE;
        }
        int percentOfFullScale = (currentLimit * 100) / this->motorProperties->I_FULL_SCALE;
        int fiveBitValue = (percentOfFullScale*100) * 31 / 10000;

        if ((fiveBitValue & (1<<0)) != 0)
        {
            this->I0Gpio.setValue(RVR::GpioValue::HIGH); // set gpio for I0 to high
        }
        else
        {
            this->I0Gpio.setValue(GpioValue::LOW); // set gpio for I0 to low
        }
        if ((fiveBitValue & (1<<1)) != 0)
        {
            this->I1Gpio.setValue(GpioValue::HIGH); // set gpio for I1 to high
        }
        else
        {
            this->I1Gpio.setValue(GpioValue::LOW); // set gpio for I1 to low
        }
        if ((fiveBitValue & (1<<2)) != 0)
        {
            this->I2Gpio.setValue(GpioValue::HIGH); // set gpio for I2 to high
        }
        else
        {
            this->I2Gpio.setValue(GpioValue::LOW); // set gpio for I2 to low
        }
        if ((fiveBitValue & (1<<3)) != 0)
        {
            this->I3Gpio.setValue(GpioValue::HIGH); // set gpio for I3 to high
        }
        else
        {
            this->I3Gpio.setValue(GpioValue::LOW); // set gpio for I3 to low
        }
        if ((fiveBitValue & (1<<4)) != 0)
        {
            this->I4Gpio.setValue(GpioValue::HIGH); // set gpio for I4 to high
        }
        else
        {
            this->I4Gpio.setValue(GpioValue::LOW); // set gpio for I4 to low
        }

        return 0;
    }

    int Motor::setupPins()
    {
        this->In1Pwm = RVR::PwmPin(this->motorProperties->IN1);
        this->In2Pwm = RVR::PwmPin(this->motorProperties->IN2);
        this->I0Gpio = RVR::GpioPin(this->motorProperties->I0, GpioDirection::OUT);
        this->I1Gpio = RVR::GpioPin(this->motorProperties->I1, GpioDirection::OUT);
        this->I2Gpio = RVR::GpioPin(this->motorProperties->I2, GpioDirection::OUT);
        this->I3Gpio = RVR::GpioPin(this->motorProperties->I3, GpioDirection::OUT);
        this->I4Gpio = RVR::GpioPin(this->motorProperties->I4, GpioDirection::OUT);
        this->FaultGpio = RVR::GpioPin(this->motorProperties->FAULT, GpioDirection::OUT);
        this->SleepGpio = RVR::GpioPin(this->motorProperties->SLEEP, GpioDirection::OUT);
        this->ResetGpio = RVR::GpioPin(this->motorProperties->RESET, GpioDirection::OUT);
        this->DecayGpio = RVR::GpioPin(this->motorProperties->DECAY, GpioDirection::OUT);

        return 0;
    }
}
