//
// Created by Bryce Carter on 8/25/15.
//

#ifndef FIRMWARE_MOTORS_H
#define FIRMWARE_MOTORS_H

#include <string>
#include "rPower.h"
#include "pins.h"
#include <chrono>
#include <unistd.h>
#include <stdexcept>
#include "../rCore/easylogging++.h"

namespace RVR
{
    // Representations of the motors in the rover product
    enum MotorName
    {
        DRIVE_MOTOR_A, // Wheel drive motor #1
        DRIVE_MOTOR_B, // Wheel drive motor #2
        CAMERA_MOTOR, // Camera articulation motor
        TREAT_MOTOR // Treat dispenser motor
    };

    // Representations of directions that a motor can spin
    enum class MotorDirection
    {
        FORWARD,
        REVERSE
    };

    // Representations of the states which the motor can be in
    enum class MotorState
    {
        STOPPED,
        RUNNING,
        SLEEP,
        RESET,
        ERROR
    };

    // Representation of the decay modes which the motor drivers can have
    enum class MotorDecayMode
    {
        FAST, // Don't break
        SLOW // Break
    };

    // Base class for motor objects
    class DRV88XXMotor
    {
    protected:

        int FAULT; // pin value for FAULT
        int SLEEP; // pin value for SLEEP
        int RESET; // pin value for RESET
        int DECAY; // pin value for DECAY
        RVR::PowerRail *POWER_RAIL; // pointer to the power rail that the motor runs off of
        int V_REF; // value (in mV) of the reference voltage
        int R_SENSE; // value (in mOhm) of the sense resistor
        int I_FULL_SCALE; // maximum value (in mA) of the chopping current

    public:
        std::string name;

        DRV88XXMotor() { }

        DRV88XXMotor(int fault, int sleep, int reset, int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name);

        //! Sets the current limit of the motor
        /*!
          \param current limit (in mA)
        */
        virtual void setCurrentLimit(int currentLimit) = 0;

        virtual int stopMotor() = 0;

        void reset();

        void sleep();

        int wake();

        void setDecay(MotorDecayMode decayMode);

        bool readFault();

    protected:
        MotorState state;

        GpioPin *FaultGpio; // A pin object that allows reading from the FAULT pin the the motor controller
        GpioPin *SleepGpio; // A pin object that allows control of the SLEEP pin the the motor controller
        GpioPin *ResetGpio; // A pin object that allows control of the RESET pin the the motor controller
        GpioPin *DecayGpio; // A pin object that allows control of the DECAY pin the the motor controller

    };

    // Subclass to represent a stepper motor (controlled by the DRV8843 chip)
    class DRV8843Motor : public DRV88XXMotor
    {
    private:
        void init(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1);

        PwmPin *aIn1Pwm; // A pin object that allows control of the BIN1 pin of the motor controller
        GpioPin *aIn2Gpio; // A pin object that allows control of the BIN2 pin of the motor controller
        PwmPin *bIn1Pwm; // A pin object that allows control of the BIN1 pin of the motor controller
        GpioPin *bIn2Gpio; // A pin object that allows control of the BIN2 pin of the motor controller
        GpioPin *aI0Gpio; // A pin object that allows control of the AI0 pin of the motor controller
        GpioPin *aI1Gpio; // A pin object that allows control of the AI1 pin of the motor controller
        GpioPin *bI0Gpio; // A pin object that allows control of the BI0 pin of the motor controller
        GpioPin *bI1Gpio; // A pin object that allows control of the BI1 pin of the motor controller

    public:

        //! All arguments constructor
        /*!
          \param powerRail pointer to the power rail that the motor runs off of
          \param vRef value (in mV) of the reference voltage
          \param vRef value (in mOhm) of the sense resistor
        */
        DRV8843Motor(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1, int fault, int sleep, int reset,
                          int decay, RVR::PowerRail *powerRail, int vRef, int rSense);
        DRV8843Motor(int aIn1, int aIn2, int bIn1, int bIn2, int aI0, int aI1, int bI0, int bI1, int fault, int sleep, int reset,
                     int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name);

        void setCurrentLimit(int currentLimit);
        int startMotor(){return -1;};
        int stopMotor(){return -1;};

    };

    // Subclass to represent a DC motor (controlled by the DRV8842 chip)
    class DRV8842Motor : public DRV88XXMotor
    {
    private:
        void init(int in1, int in2, int i0, int i1, int i2, int i3, int i4);

        PwmPin *In1Pwm; // A pin object that allows control of the IN1 pin of the motor controller
        GpioPin *In2Gpio; // A pin object that allows control of the IN2 pin of the motor controller
        GpioPin *I0Gpio; // A pin object that allows control of the I0 pin of the motor controller
        GpioPin *I1Gpio; // A pin object that allows control of the I1 pin of the motor controller
        GpioPin *I2Gpio; // A pin object that allows control of the I2 pin of the motor controller
        GpioPin *I3Gpio; // A pin object that allows control of the I3 pin of the motor controller
        GpioPin *I4Gpio; // A pin object that allows control of the I4 pin of the motor controller

        unsigned int rampTime = 0;
    public:
        //! All arguments constructor
        /*!
          \param powerRail pointer to the power rail that the motor runs off of
          \param vRef value (in mV) of the reference voltage
          \param vRef value (in mOhm) of the sense resistor
        */
        DRV8842Motor(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset,
                     int decay, RVR::PowerRail *powerRail, int vRef, int rSense);
        DRV8842Motor(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset,
                     int decay, RVR::PowerRail *powerRail, int vRef, int rSense, std::string name);

        void setCurrentLimit(int currentLimit);

        // Sets the ramp time in milliseconds
        int setRampTime(unsigned int rampTime_ms);

        // Target speed should be an integer between 1 and 100
        int startMotor(int targetSpeedPercent, MotorDirection direction);

        double getSpeed();

        int stopMotor();
    };

}
#endif //FIRMWARE_MOTORS_H
