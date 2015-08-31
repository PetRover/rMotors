//
// Created by Bryce Carter on 8/25/15.
//

#include "rMotors.h"

namespace RVR
{

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

    RVR::Motor::Motor(RVR::MotorName motorName)
    {
        // Assign the correct pin mapping for the motor
        switch (motorName)
        {
            case RVR::DRIVE_MOTOR_1:
                this->motorProperties = RVR::Motor::drive1MotorMapping;
            case RVR::DRIVE_MOTOR_2:
                this->motorProperties = RVR::Motor::drive2MotorMapping;
            case RVR::TREAT_MOTOR:
                this->motorProperties = RVR::Motor::treatMotorMapping;
            case RVR::CAMERA_MOTOR:
                this->motorProperties = RVR::Motor::cameraMotorMapping;
        }
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
            // set gpio for I0 to high
        }
        else
        {
            // set gpio for I0 to low
        }
        if ((fiveBitValue & (1<<1)) != 0)
        {
            // set gpio for I1 to high
        }
        else
        {
            // set gpio for I1 to low
        }
        if ((fiveBitValue & (1<<2)) != 0)
        {
            // set gpio for I2 to high
        }
        else
        {
            // set gpio for I2 to low
        }
        if ((fiveBitValue & (1<<3)) != 0)
        {
            // set gpio for I3 to high
        }
        else
        {
            // set gpio for I3 to low
        }
        if ((fiveBitValue & (1<<4)) != 0)
        {
            // set gpio for I4 to high
        }
        else
        {
            // set gpio for I4 to low
        }

        return 0;
    }
}
