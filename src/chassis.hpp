#pragma once

#include <iostream>
#include <vector>
#include <ratio>

#include "main.h"

#include "units.hpp"

using namespace units;

enum Gearsets
{
    RED = 0,
    GREEN = 1,
    BLUE = 2
};

// getValue() will return the value relative to meters, which is not what I want

class Chassis {
public:
    Chassis(std::vector<pros::Motor*> motors, Gearsets gearset = Gearsets::GREEN, int wheelDiameter = 12.56,
        int wheelTrack = 24, std::ratio<1, 1> gearRatio)
        : motors(motors), gearset(gearset), wheelDiameter(wheelDiameter), wheelTrack(wheelTrack), gearRatio(gearRatio)
    { feetToWheelRotations = units::foot.getValue() / (wheelDiameter * M_PI); }

    void forward(units::Length length, int speed)
    {
        units::Length currentPosition = 0_in;
        int previousPosition = 0;
        double error = length.getValue() - currentPosition.getValue();
        double integral = 0;
        double derivative = 0;
        double previousError = error;

        while (abs(error) > 0.01)
        {
            previousPosition = motors[0]->get_position();
            error = length.getValue() - currentPosition.getValue();
            integral += error;
            derivative = error - previousError;

            int output = kP * error + kI * integral + kD * derivative;

            setMotorSpeed(output);

            previousError = error;
            currentPosition += wheelRotationsToInches(motors[0]->get_position() - previousPosition);
        }

        setMotorSpeed(0); // stop the motor when the target position is reached
    }

    void backward(units::Length length, int speed);
    void turn(units::Angle angle, int speed);

    void setMotorSpeed(int speed)
    {
        for (auto motor : motors)
        {
            motor->move(speed);
        }
    }

    inline int wheelRotationsToInches(int rotations)
    {
        return rotations / (wheelDiameter * M_PI);
    }

    std::vector<pros::Motor*> motors;
    int kP, kI, kD;
    Gearsets gearset;
    int wheelDiameter; // Inches
    int wheelTrack; // Inches
    std::ratio<1, 1> gearRatio;

    int feetToWheelRotations;

};