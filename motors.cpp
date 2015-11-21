//
// Created by Bryce Carter on 8/25/15.
//

#include "rMotors.h"

namespace RVR
{
    // ==============================================================
    // DRV88XXMotor Class Member functions
    // ==============================================================

    DRV88XXMotor::DRV88XXMotor(int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name)
    {
        this->POWER_RAIL = powerRail;
        this->V_REF = vRef;
        this->R_SENSE = rSense;
        this->I_FULL_SCALE = this->V_REF / (5 * this->R_SENSE) * 1000;

        this->name = name;

        this->FaultGpio = new GpioPin(fault);
        this->SleepGpio = new GpioPin(sleep, GpioDirection::OUT);
        this->ResetGpio = new GpioPin(reset, GpioDirection::OUT);
        this->DecayGpio = new GpioPin(decay, GpioDirection::OUT);

    }

    void DRV88XXMotor::reset()
    {
        VLOG(1) << "Resetting motor <" << this->name << ">";
        this->state = MotorState::RESET;
        this->ResetGpio->setValue(GpioValue::LOW);
        usleep(2000);
        this->stopMotor();
        this->ResetGpio->setValue(GpioValue::HIGH);
        usleep(2000);
        this->state = MotorState::STOPPED;
        VLOG(1) << "Motor is now reset and in STOPPED state <" << this->name << ">";
    }

    void DRV88XXMotor::sleep()
    {
        VLOG(1) << "Putting motor to sleep <" << this->name << ">";
        this->state = MotorState::SLEEP;
        this->SleepGpio->setValue(GpioValue::LOW);
        this->stopMotor();
        VLOG(1) << "Motor is not asleep and in the STOPPED state <" << this->name << ">";
    }

    int DRV88XXMotor::wake()
    {
        VLOG(1) << "Waking motor up <" << this->name << ">";
        if (this->state == MotorState::SLEEP)
        {
            this->SleepGpio->setValue(GpioValue::HIGH);
            usleep(2000);
            this->state = MotorState::STOPPED;
            VLOG(1) << "Motor is now awake <" << this->name << ">";
            return 0;
        }
        else
        {
            LOG(WARNING) << "Wake did nto occur because the motor was not in a valid state <" << this->name << ">";
            return 1; // The wake did not occur because the motor is not in a valid state
        }
    }

    void DRV88XXMotor::setDecay(MotorDecayMode decayMode)
    {
        switch (decayMode)
        {
            case MotorDecayMode::FAST:
                VLOG(1) << "Setting motor decay mode to 'FAST' <" << this->name << ">";
                this->DecayGpio->setValue(GpioValue::HIGH);
                break;
            case MotorDecayMode::SLOW:
                VLOG(1) << "Setting motor decay mode to 'SLOW' <" << this->name << ">";
                this->DecayGpio->setValue(GpioValue::LOW);
                break;
        }
    }

    bool DRV88XXMotor::readFault()
    {
        VLOG(3) << "Reading fault from motor <" << this->name << ">";
        GpioValue faultBit = this->FaultGpio->getValue();
        switch (faultBit)
        {
            case GpioValue::HIGH:
                VLOG(3) << "No fault found <" << this->name << ">";
                return false;
            case GpioValue::LOW:
                LOG(WARNING) << "Motor had a fault... Please investigate <" << this->name << ">";
                return true;
            case GpioValue::ERROR:
                throw std::runtime_error("read from fault bit on motor driver failed");
        }
    }


    int DRV88XXMotor::getMaxCurrent()
    {
        return this->I_FULL_SCALE;
    }

    // ==============================================================
    // DRV8843Motor Class Member functions
    // ==============================================================


    void DRV8843Motor::init(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1)
    {
        VLOG(1) << "Initializing motor [STEPPER] <" << this->name << ">";

        this->aIn1Pwm = new PwmPin(aIn1);
        this->aIn2Gpio = new GpioPin(aIn2, GpioDirection::OUT);
        this->bIn1Pwm = new PwmPin(bIn1);
        this->bIn2Gpio = new GpioPin(bIn2, GpioDirection::OUT);
        this->aI0Gpio = new GpioPin(aI0, GpioDirection::OUT);
        this->aI1Gpio = new GpioPin(aI1, GpioDirection::OUT);
        this->bI1Gpio = new GpioPin(bI0, GpioDirection::OUT);
        this->bI0Gpio = new GpioPin(bI1, GpioDirection::OUT);

        this->aIn1Pwm->setEnable(false);
        this->bIn1Pwm->setEnable(false);

        this->reset();
        this->sleep();

        VLOG(1) << "Motor initialized [STEPPER] <" << this->name << ">";
    }

    DRV8843Motor::DRV8843Motor(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1, int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense)
    : DRV88XXMotor(fault, sleep, reset, decay, powerRail, vRef, rSense, "NONE")
    {
        this->init(aIn1, aIn2, bIn1, bIn2, aI0, aI1, bI0, bI1);
    }

    DRV8843Motor::DRV8843Motor(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1, int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name)
    : DRV88XXMotor(fault, sleep, reset, decay, powerRail, vRef, rSense, name)
    {
        this->init(aIn1, aIn2, bIn1, bIn2, aI0, aI1, bI0, bI1);
    }


    void DRV8843Motor::setCurrentLimit(int currentLimit)
    {

        if (currentLimit > this->I_FULL_SCALE)
        {
            currentLimit = this->I_FULL_SCALE;
        }
        int percentOfFullScale = (currentLimit * 100) / this->I_FULL_SCALE;
        VLOG(1) << "Setting motor current limit to" << percentOfFullScale << "% (" << currentLimit << "mA) <" <<
                this->name << ">";
        int twoBitValue = (percentOfFullScale * 100) * 3 / 10000;

        if ((twoBitValue & (1 << 0)) != 0)
        {
            VLOG(3) << "Settign aI0 and bI0 HIGH <" << this->name << ">";
            this->aI0Gpio->setValue(GpioValue::HIGH); // set gpio for aI0 to high
            this->bI0Gpio->setValue(GpioValue::HIGH); // set gpio for bI0 to high
        }
        else
        {
            VLOG(3) << "Settign aI0 and bI0 LOW <" << this->name << ">";
            this->aI0Gpio->setValue(GpioValue::LOW); // set gpio for aI0 to low
            this->bI0Gpio->setValue(GpioValue::LOW); // set gpio for bI0 to low
        }
        if ((twoBitValue & (1 << 1)) != 0)
        {
            VLOG(3) << "Settign aI1 and bI1 HIGH <" << this->name << ">";
            this->aI1Gpio->setValue(GpioValue::HIGH); // set gpio for aI1 to high
            this->bI1Gpio->setValue(GpioValue::HIGH); // set gpio for bI1 to high
        }
        else
        {
            VLOG(3) << "Settign aI1 and bI1 LOW <" << this->name << ">";
            this->aI1Gpio->setValue(GpioValue::LOW); // set gpio for aI1 to low
            this->bI1Gpio->setValue(GpioValue::LOW); // set gpio for bI1 to low
        }
    }

    void DRV8843Motor::enableMotor()
    {
        this->aIn1Pwm->setEnable(true);
        this->bIn1Pwm->setEnable(true);
        this->step(MotorDirection::FORWARD);
        this->step(MotorDirection::REVERSE);
    }

    void DRV8843Motor::step(MotorDirection direction)
    {
        int increment;
        switch (direction)
        {
            case MotorDirection::FORWARD:
                increment = 1;
                break;
            case MotorDirection::REVERSE:
                increment = -1;
                break;
        }

        this->currentState = (this->currentState + increment) % 4;

        switch (currentState)
        {
            case 0:
                this->aIn1Pwm->setDutyCyclePercent(100);
                this->aIn2Gpio->setValue(GpioValue::LOW);
                this->bIn1Pwm->setDutyCyclePercent(100);
                this->bIn2Gpio->setValue(GpioValue::LOW);
                break;
            case 1:
                this->aIn1Pwm->setDutyCyclePercent(0);
                this->aIn2Gpio->setValue(GpioValue::HIGH);
                this->bIn1Pwm->setDutyCyclePercent(100);
                this->bIn2Gpio->setValue(GpioValue::LOW);
                break;
            case 2:
                this->aIn1Pwm->setDutyCyclePercent(0);
                this->aIn2Gpio->setValue(GpioValue::HIGH);
                this->bIn1Pwm->setDutyCyclePercent(0);
                this->bIn2Gpio->setValue(GpioValue::HIGH);
                break;
            case 3:
                this->aIn1Pwm->setDutyCyclePercent(100);
                this->aIn2Gpio->setValue(GpioValue::LOW);
                this->bIn1Pwm->setDutyCyclePercent(0);
                this->bIn2Gpio->setValue(GpioValue::HIGH);
                break;
            default:
                throw(std::runtime_error("invaid current state of stepper motor"));
        }
    }

    // ==============================================================
    // DRV8842Motor Class Member functions
    // ==============================================================
    void DRV8842Motor::init(int in1, int in2, int i0, int i1, int i2, int i3, int i4)
    {
        VLOG(1) << "Initializing motor [DC] <" << this->name << ">";
        this->In1Pwm = new PwmPin(in1);
        this->In2Gpio = new GpioPin(in2, GpioDirection::OUT);
        this->I0Gpio = new GpioPin(i0, GpioDirection::OUT);
        this->I1Gpio = new GpioPin(i1, GpioDirection::OUT);
        this->I2Gpio = new GpioPin(i2, GpioDirection::OUT);
        this->I3Gpio = new GpioPin(i3, GpioDirection::OUT);
        this->I4Gpio = new GpioPin(i4, GpioDirection::OUT);

        this->In1Pwm->setEnable(false);

        this->reset();
        this->sleep();
        VLOG(1) << "Motor initialized [DC] <" << name << ">";
    }

    DRV8842Motor::DRV8842Motor(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense)
    : DRV88XXMotor(fault, sleep, reset, decay, powerRail, vRef, rSense, "NONE")
    {
        this->init(in1, in2, i0, i1, i2, i3, i4);
    }

    DRV8842Motor::DRV8842Motor(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name)
    : DRV88XXMotor(fault, sleep, reset, decay, powerRail, vRef, rSense, name)
    {
        this->init(in1, in2, i0, i1, i2, i3, i4);
    }

    void DRV8842Motor::setCurrentLimit(int currentLimit)
    {
        if (currentLimit > this->I_FULL_SCALE)
        {
            currentLimit = this->I_FULL_SCALE;
        }
        int percentOfFullScale = (currentLimit * 100) / this->I_FULL_SCALE;
        VLOG(1) << "Setting motor current limit to " << percentOfFullScale << "% (" << currentLimit << "mA) <" <<
                this->name << ">";
        int fiveBitValue = (percentOfFullScale * 100) * 31 / 10000;

        if ((fiveBitValue & (1 << 0)) != 0)
        {
            VLOG(3) << "Settign I0 HIGH <" << this->name << ">";
            this->I0Gpio->setValue(GpioValue::HIGH); // set gpio for I0 to high
        }
        else
        {
            VLOG(3) << "Settign I0 LOW <" << this->name << ">";
            this->I0Gpio->setValue(GpioValue::LOW); // set gpio for I0 to low
        }
        if ((fiveBitValue & (1 << 1)) != 0)
        {
            VLOG(3) << "Settign I1 HIGH <" << this->name << ">";
            this->I1Gpio->setValue(GpioValue::HIGH); // set gpio for I1 to high
        }
        else
        {
            VLOG(3) << "Settign I1 LOW <" << this->name << ">";
            this->I1Gpio->setValue(GpioValue::LOW); // set gpio for I1 to low
        }
        if ((fiveBitValue & (1 << 2)) != 0)
        {
            VLOG(3) << "Settign I2 HIGH <" << this->name << ">";
            this->I2Gpio->setValue(GpioValue::HIGH); // set gpio for I2 to high
        }
        else
        {
            VLOG(3) << "Settign I2 LOW <" << this->name << ">";
            this->I2Gpio->setValue(GpioValue::LOW); // set gpio for I2 to low
        }
        if ((fiveBitValue & (1 << 3)) != 0)
        {
            VLOG(3) << "Settign I3 HIGH <" << this->name << ">";
            this->I3Gpio->setValue(GpioValue::HIGH); // set gpio for I3 to high
        }
        else
        {
            VLOG(3) << "Settign I3 LOW <" << this->name << ">";
            this->I3Gpio->setValue(GpioValue::LOW); // set gpio for I3 to low
        }
        if ((fiveBitValue & (1 << 4)) != 0)
        {
            VLOG(3) << "Settign I4 HIGH <" << this->name << ">";
            this->I4Gpio->setValue(GpioValue::HIGH); // set gpio for I4 to high
        }
        else
        {
            VLOG(3) << "Settign I4 LOW <" << this->name << ">";
            this->I4Gpio->setValue(GpioValue::LOW); // set gpio for I4 to low
        }
    }

    int DRV8842Motor::setRampTime(unsigned int rampTime_ms)
    {
        VLOG(1) << "Setting motor ramp time to " << rampTime_ms << " (ms) <" << this->name << ">";
        this->rampTime = rampTime_ms;

        return 0;
    }

    int DRV8842Motor::startMotor(int targetSpeedPercent, MotorDirection direction)
    {
        VLOG(1) << "Starting motor <" << this->name << ">";
        if (this->state != MotorState::STOPPED)
        {
            LOG(WARNING) << "Motor failed to start because the state was invlaid <" << this->name << ">";
            return 1; // 1 means that the motor was not started because it is currently in an invalid state
        }

        double pwmStartPercent;
        GpioValue gpioInValue;
        int rampPolarity;
        switch (direction)
        {
            case MotorDirection::FORWARD:
                VLOG(2) << "Motor direction will be FORWARD <" << this->name << ">";
                pwmStartPercent = 0;
                gpioInValue = GpioValue::LOW;
                rampPolarity = 1;
                break;
            case MotorDirection::REVERSE:
                VLOG(2) << "Motor direction will be REVERSE <" << this->name << ">";
                pwmStartPercent = 100;
                gpioInValue = GpioValue::HIGH;
                rampPolarity = -1;
                break;
        }


        double pwmPercentPer_ms; // Number of PWM duty cycle percent to increase every millisecond

        // Prevent division by zero when ramp time is zero
        if (this->rampTime > 0)
        {
            pwmPercentPer_ms = (targetSpeedPercent / (double) this->rampTime);
        }
        else
        {
            pwmPercentPer_ms = targetSpeedPercent;
        }

        this->In1Pwm->setDutyCyclePercent(pwmStartPercent);
        this->In1Pwm->setEnable(true);
        this->In2Gpio->setValue(gpioInValue);

        int loopCount = 0;
        std::chrono::duration<double> rampTimeElapsed;

        std::chrono::high_resolution_clock::time_point rampStartTime = std::chrono::high_resolution_clock::now();
        double pwmPercent;

        VLOG(2) << "Ramping up motor speed for " << this->rampTime << " (ms) <" << this->name << ">";
        do // Note, no print functions inside this loop because they will slow it down
        {
            loopCount++;
            rampTimeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::high_resolution_clock::now() - rampStartTime);
            pwmPercent = pwmStartPercent + rampPolarity * (rampTimeElapsed.count() * 1000) * pwmPercentPer_ms;

            this->In1Pwm->setDutyCyclePercent(pwmPercent);

            int error_ms = loopCount - (int) (rampTimeElapsed.count() * 1000);
            if (error_ms > 0)
            {
                usleep(1000 * (unsigned int) (error_ms));
            }
        }
        while (rampTimeElapsed.count() * 1000 < this->rampTime);

        this->In1Pwm->setDutyCyclePercent(pwmStartPercent + rampPolarity * targetSpeedPercent);

        this->state = MotorState::RUNNING;
        VLOG(1) << "Motor is now running <" << this->name << ">";
        return 0;
    }

    int DRV8842Motor::stopMotor()
    {
        VLOG(1) << "Stopping motor <" << this->name << ">";
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

    double DRV8842Motor::getSpeed()
    {
        VLOG(1) << "Fetching motor speed <" << this->name << ">";
        double speed = (this->In1Pwm->getDutyCycleTime() * 100) / (double) this->In1Pwm->getPeriod();
        VLOG(1) << "Motor speed is currently "<<speed<<"% <" << this->name << ">";
        return speed;
    }

}