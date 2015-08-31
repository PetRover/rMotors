//
// Created by Bryce Carter on 8/25/15.
//

#ifndef FIRMWARE_MOTORS_H
#define FIRMWARE_MOTORS_H

#include <string>
#include "rPower.h"

namespace RVR
{
    enum MotorName
    {
        DRIVE_MOTOR_1,
        DRIVE_MOTOR_2,
        CAMERA_MOTOR,
        TREAT_MOTOR
    };

    class MotorProperties
    {
    public:
        int IN1; // pin value for IN1
        int IN2; // pin value for IN2
        int I0; // pin value for I0
        int I1; // pin value for I1
        int I2; // pin value for I2
        int I3; // pin value for I3
        int I4; // pin value for I4
        int FAULT; // pin value for FAULT
        int SLEEP; // pin value for SLEEP
        int RESET; // pin value for RESET
        int DECAY; // pin value for DECAY
        RVR::PowerRail * POWER_RAIL; // pointer to the power rail that the motor runs off of
        int V_REF; // value (in mV) of the reference voltage
        int R_SENSE; // value (in mOhm) of the sense resistor
        int I_FULL_SCALE; // maximum value (in mA) of the chopping current

        MotorProperties(){}

        //! All arguments constructor
        /*!
          \param in1 pin value for IN1
          \param in2 pin value for IN2
          \param i0 pin value for I0
          \param i1 pin value for I1
          \param i2 pin value for I2
          \param i3 pin value for I3
          \param i4 pin value for I4
          \param fault pin value for FAULT
          \param sleep pin value for SLEEP
          \param reset pin value for RESET
          \param decay pin value for DECAY
          \param powerRail pointer to the power rail that the motor runs off of
          \param vRef value (in mV) of the reference voltage
          \param vRef value (in mOhm) of the sense resistor
        */
        MotorProperties(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset, int decay, RVR::PowerRail * powerRail, int vRef, int rSense);
    };


    class Motor
    {
    public:
        Motor(){}
        Motor(MotorName motorName);

        //! Sets the current limit of the motor
        /*!
          \param current limit (in mA)
        */
        int setCurrentLimit(int currentLimit);

    protected:
        const MotorProperties *motorProperties;

        static const MotorProperties * const drive1MotorMapping;
        static const MotorProperties * const drive2MotorMapping;
        static const MotorProperties * const treatMotorMapping;
        static const MotorProperties * const cameraMotorMapping;

    };

    class StepperMotor : public Motor
    {

    };

    class DcMotor : public Motor
    {

    };

}
#endif //FIRMWARE_MOTORS_H
