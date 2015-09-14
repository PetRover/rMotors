//
// Created by Bryce Carter on 8/25/15.
//

#ifndef FIRMWARE_MOTORS_H
#define FIRMWARE_MOTORS_H

#include <string>
#include "rPower.h"
#include "pins.h"

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

    // A data structure that contains all of the information to describe the connections to a given motor in the system
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
        RVR::PowerRail *POWER_RAIL; // pointer to the power rail that the motor runs off of
        int V_REF; // value (in mV) of the reference voltage
        int R_SENSE; // value (in mOhm) of the sense resistor
        int I_FULL_SCALE; // maximum value (in mA) of the chopping current

        MotorProperties() { }

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
        MotorProperties(int in1, int in2, int i0, int i1, int i2, int i3, int i4, int fault, int sleep, int reset,
                        int decay, RVR::PowerRail *powerRail, int vRef, int rSense);
    };


    // Base class for motor objects
    class Motor
    {
    private:
        // The fixed properties for each motor in the rover product
        static const MotorProperties *const driveAMotorMapping;
        static const MotorProperties *const driveBMotorMapping;
        static const MotorProperties *const treatMotorMapping;
        static const MotorProperties *const cameraMotorMapping;

        // Helper function to set up the pin objects used for interfacing with the motor controller
        int setupPins();

    public:
        Motor() { }

        // Initializes a motor object corresponding to the motor name given
        Motor(MotorName motorName);

        //! Sets the current limit of the motor
        /*!
          \param current limit (in mA)
        */
        void setCurrentLimit(int currentLimit);

        virtual int stopMotor() = 0;

        void reset();

        void sleep();

        int wake();

        void setDecay(MotorDecayMode decayMode);

        bool readFault();

    protected:
        MotorState state;

        // stores the motor properties for the specific motor which an instance of this class represents
        const MotorProperties *motorProperties;

        PwmPin *In1Pwm; // A pin object that allows control of the IN1 pin of the motor controller
        GpioPin *In2Gpio; // A pin object that allows control of the IN2 pin of the motor controller
        GpioPin *I0Gpio; // A pin object that allows control of the I0 pin of the motor controller
        GpioPin *I1Gpio; // A pin object that allows control of the I1 pin of the motor controller
        GpioPin *I2Gpio; // A pin object that allows control of the I2 pin of the motor controller
        GpioPin *I3Gpio; // A pin object that allows control of the I3 pin of the motor controller
        GpioPin *I4Gpio; // A pin object that allows control of the I4 pin of the motor controller
        GpioPin *FaultGpio; // A pin object that allows reading from the FAULT pin the the motor controller
        GpioPin *SleepGpio; // A pin object that allows control of the SLEEP pin the the motor controller
        GpioPin *ResetGpio; // A pin object that allows control of the RESET pin the the motor controller
        GpioPin *DecayGpio; // A pin object that allows control of the DECAY pin the the motor controller

    };

    // Subclass to represent a stepper motor (controlled by the DRV8843 chip)
    class StepperMotor : public Motor
    {

    };

    // Subclass to represent a DC motor (controlled by the DRV8842 chip)
    class DcMotor : public Motor
    {
    private:
        unsigned int rampTime = 0;
    public:
        DcMotor(MotorName motorName) : Motor(motorName) { };

        // Sets the ramp time in milliseconds
        int setRampTime(unsigned int rampTime_ms);

        // Target speed should be an integer between 1 and 100
        int startMotor(int targetSpeedPercent, MotorDirection direction);

        int stopMotor();
    };

}
#endif //FIRMWARE_MOTORS_H
