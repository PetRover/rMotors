//
// Created by Bryce Carter on 8/25/15.
//

#include "rMotors.h"
#include <chrono>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedImportStatement"
#include <unistd.h>
#include <stdexcept>


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
            case RVR::DRIVE_MOTOR_A:
                this->motorProperties = RVR::Motor::driveAMotorMapping;
                break;
            case RVR::DRIVE_MOTOR_B:
                this->motorProperties = RVR::Motor::driveBMotorMapping;
                break;
            case RVR::TREAT_MOTOR:
                this->motorProperties = RVR::Motor::treatMotorMapping;
                break;
            case RVR::CAMERA_MOTOR:
                this->motorProperties = RVR::Motor::cameraMotorMapping;
                break;
        }
        this->setupPins();

        this->In1Pwm->setEnable(false);
        this->In1Pwm->setPeriod(20000); // F_PWM recommended between 0 and 100KHz
        this->state = MotorState::STOPPED;
    }

    RVR::PowerRail* motorRail = RVR::PowerManager::getRail(RVR::RAIL12V0);
    const RVR::MotorProperties *const RVR::Motor::driveAMotorMapping = new RVR::MotorProperties(13, 35, 7, 6, 5, 4, 3,
                                                                                                9, 10, 11, 8, motorRail,
                                                                                                2500,
                                                                                                250); //old (36, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, motorRail, 2500, 250);
    const RVR::MotorProperties *const RVR::Motor::driveBMotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8,
                                                                                                9, 10, 11, motorRail,
                                                                                                2500, 250);
    const RVR::MotorProperties * const RVR::Motor::treatMotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);
    const RVR::MotorProperties * const RVR::Motor::cameraMotorMapping = new RVR::MotorProperties(1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, motorRail, 2500, 250);

    void Motor::setCurrentLimit(int currentLimit)
    {
        if (currentLimit > this->motorProperties->I_FULL_SCALE)
        {
            currentLimit = this->motorProperties->I_FULL_SCALE;
        }
        int percentOfFullScale = (currentLimit * 100) / this->motorProperties->I_FULL_SCALE;
        int fiveBitValue = (percentOfFullScale*100) * 31 / 10000;

        if ((fiveBitValue & (1<<0)) != 0)
        {
            this->I0Gpio->setValue(RVR::GpioValue::HIGH); // set gpio for I0 to high
        }
        else
        {
            this->I0Gpio->setValue(GpioValue::LOW); // set gpio for I0 to low
        }
        if ((fiveBitValue & (1<<1)) != 0)
        {
            this->I1Gpio->setValue(GpioValue::HIGH); // set gpio for I1 to high
        }
        else
        {
            this->I1Gpio->setValue(GpioValue::LOW); // set gpio for I1 to low
        }
        if ((fiveBitValue & (1<<2)) != 0)
        {
            this->I2Gpio->setValue(GpioValue::HIGH); // set gpio for I2 to high
        }
        else
        {
            this->I2Gpio->setValue(GpioValue::LOW); // set gpio for I2 to low
        }
        if ((fiveBitValue & (1<<3)) != 0)
        {
            this->I3Gpio->setValue(GpioValue::HIGH); // set gpio for I3 to high
        }
        else
        {
            this->I3Gpio->setValue(GpioValue::LOW); // set gpio for I3 to low
        }
        if ((fiveBitValue & (1<<4)) != 0)
        {
            this->I4Gpio->setValue(GpioValue::HIGH); // set gpio for I4 to high
        }
        else
        {
            this->I4Gpio->setValue(GpioValue::LOW); // set gpio for I4 to low
        }
    }


    void Motor::reset()
    {
        this->state = MotorState::RESET;
        this->ResetGpio->setValue(GpioValue::LOW);
        usleep(2000);
        this->stopMotor();
        this->ResetGpio->setValue(GpioValue::HIGH);
        usleep(2000);
        this->state = MotorState::STOPPED;
    }

    void Motor::sleep()
    {
        this->state = MotorState::SLEEP;
        this->SleepGpio->setValue(GpioValue::LOW);
        this->stopMotor();
    }

    int Motor::wake()
    {
        if (this->state == MotorState::SLEEP)
        {
            this->SleepGpio->setValue(GpioValue::HIGH);
            usleep(2000);
            this->state = MotorState::STOPPED;
            return 0;
        }
        else
        {
            return 1; // The wake did not occur because the motor is not in a valid state
        }
    }

    void Motor::setDecay(MotorDacayMode decayMode)
    {
        switch (decayMode)
        {
            case MotorDacayMode::FAST:
                this->DecayGpio->setValue(GpioValue::HIGH);
                break;
            case MotorDacayMode::SLOW:
                this->DecayGpio->setValue(GpioValue::LOW);
                break;
        }
    }

    bool Motor::readFault()
    {
        GpioValue faultBit = this->FaultGpio->getValue();
        switch (faultBit)
        {
            case GpioValue::HIGH:
                return false;
                break;
            case GpioValue::LOW:
                return true;
                break;
            case GpioValue::ERROR:
                throw std::runtime_error("read from fault bit on motor driver failed");
                break;
        }
    }

    int Motor::setupPins()
    {
        this->In1Pwm = new RVR::PwmPin(this->motorProperties->IN1);
        this->In2Gpio = new RVR::GpioPin(this->motorProperties->IN2, GpioDirection::OUT);
        this->I0Gpio = new RVR::GpioPin(this->motorProperties->I0, GpioDirection::OUT);
        this->I1Gpio = new RVR::GpioPin(this->motorProperties->I1, GpioDirection::OUT);
        this->I2Gpio = new RVR::GpioPin(this->motorProperties->I2, GpioDirection::OUT);
        this->I3Gpio = new RVR::GpioPin(this->motorProperties->I3, GpioDirection::OUT);
        this->I4Gpio = new RVR::GpioPin(this->motorProperties->I4, GpioDirection::OUT);
        this->FaultGpio = new RVR::GpioPin(this->motorProperties->FAULT, GpioDirection::OUT);
        this->SleepGpio = new RVR::GpioPin(this->motorProperties->SLEEP, GpioDirection::OUT);
        this->ResetGpio = new RVR::GpioPin(this->motorProperties->RESET, GpioDirection::OUT);
        this->DecayGpio = new RVR::GpioPin(this->motorProperties->DECAY, GpioDirection::OUT);

        return 0;
    }

    int DcMotor::setRampTime(unsigned int rampTime_ms)
    {
        this->rampTime = rampTime_ms;

        return 0;
    }

    int DcMotor::startMotor(int targetSpeedPercent, MotorDirection direction)
    {
        if (this->state != MotorState::STOPPED)
        {
            return 1; // 1 means that the motor was not started because it is currently in an invalid state
        }

        double pwmInPercent;
        GpioValue gpioInValue;
        int rampPolarity;
        switch (direction)
        {
            case MotorDirection::FORWARD:
                pwmInPercent = 0;
                gpioInValue = GpioValue::LOW;
                rampPolarity = 1;
                break;
            case MotorDirection::REVERSE:
                pwmInPercent = 100;
                gpioInValue = GpioValue::HIGH;
                rampPolarity = -1;
                break;
        }


        double pwmPercentPerRampStep;

        // Prevent division by zero when ramp time is zero
        if (this->rampTime > 0)
        {
            pwmPercentPerRampStep = (targetSpeedPercent / (double) this->rampTime);
        }
        else
        {
            pwmPercentPerRampStep = targetSpeedPercent;
        }


        this->In1Pwm->setDutyCyclePercent(pwmInPercent);
        this->In1Pwm->setEnable(true);
        this->In2Gpio->setValue(gpioInValue);

        int loopCount = 0;
        std::chrono::duration<double> rampTimeElapsed;

        std::chrono::high_resolution_clock::time_point rampStartTime = std::chrono::high_resolution_clock::now();
        do // Note, no print functions inside this loop because they will slow it down
        {
            loopCount++;
            pwmInPercent += rampPolarity * pwmPercentPerRampStep;
            this->In1Pwm->setDutyCyclePercent(pwmInPercent);
            do
            {
                rampTimeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                        std::chrono::high_resolution_clock::now() - rampStartTime);
                usleep(50);
            }
            while (rampTimeElapsed.count() * 1000 < loopCount); // Each loop should take 1 ms

        }
        while (rampTimeElapsed.count() * 1000 < this->rampTime);

        this->In1Pwm->setDutyCyclePercent(targetSpeedPercent);

        this->state = MotorState::RUNNING;
        return 0;
    }

    int DcMotor::stopMotor()
    {
        if (this->state == MotorState::RUNNING)
        {
            this->In2Gpio->setValue(GpioValue::LOW);
            this->In1Pwm->setDutyCyclePercent(0);
            this->In1Pwm->setEnable(false);

            this->state = MotorState::STOPPED;
            return 0;
        }
        else
        {
            return 1; // 1 means that the motor was not stopped because it is currently in an invalid state
        }
    }
}

#pragma clang diagnostic pop