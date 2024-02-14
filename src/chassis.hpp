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

class Chassis
{
public:
    Chassis(std::vector<pros::Motor*> motors, pros::IMU* inertial, int rotation = 0, int kP = 1, int kI = 0, int kD = 0, Gearsets gearset = Gearsets::GREEN, int wheelDiameter = 12.56,
        int wheelTrack = 24, int gearRatio = 1)
        : motors(motors), inertial(inertial), gearset(gearset), wheelDiameter(wheelDiameter), wheelTrack(wheelTrack), gearRatio(gearRatio)
    {
        feetToWheelRotations = units::foot.getValue() / (wheelDiameter * M_PI);
        
        if (gearset == Gearsets::RED)
        {
            ticksPerMotorRevolution = 1800;
            maxRPM = 100;
        }
        else if (gearset == Gearsets::BLUE)
        {
            ticksPerMotorRevolution = 300;
            maxRPM = 600;
        }

        ticksPerMotorRevolution *= gearRatio;
    }

    void forward(int length, int speed) // Length in inches
    {
        if (speed > maxRPM)
            speed = maxRPM;
            
        int currentPosition = 0; // In inches
        int previousPosition = 0;
        double error = length - currentPosition;
        double integral = 0;
        double derivative = 0;
        double previousError = error;
        int speedPercent = speed / maxRPM;

        while (abs(error) > 0.01)
        {
            previousPosition = motors[0]->get_position();
            error = length - currentPosition;
            integral += error;
            derivative = error - previousError;

            int output = (kP * error) + (kI * integral) + (kD * derivative) * (speedPercent);

            setMotorSpeed(output);

            previousError = error;
            currentPosition += wheelRotationsToInches(motorTicksToRotations(motors[0]->get_position() - previousPosition));

            pros::delay(15); // 15ms dT
        }

        setMotorSpeed(0); // Stop the motor when the target position is reached
    }

    void turn(int angle, int speed)
    {
        if (speed > maxRPM)
            speed = maxRPM;
        // Calculate the direction to turn
        int currentAngle = fmod(inertial->get_rotation(), 360);
        int angleDifference = angle - fmod(inertial->get_rotation(), 360);
        if (angleDifference < 0)
            angleDifference += 180;
        else if (angleDifference > 360)
            angleDifference -= 180;

        int direction = angleDifference > 180 ? -1 : 1;
        int mappedSpeed = (speed * 127 / maxRPM);

        // Turn left motors forward and right motors backward
        for (int i = 0; i < motors.size(); i++)
        {
            if (i % 2 == 0)
                motors[i]->move(mappedSpeed * direction);
            else
                motors[i]->move(-mappedSpeed * direction);
        }

        while (abs(angle - fmod(inertial->get_rotation(), 360)) > 0.1)
        {
            pros::delay(5);
        }

        setMotorSpeed(0);
    }

    inline void setMotorSpeed(int speed)
    {
        for (pros::Motor* motor : motors)
        {
            motor->move(speed);
        }
    }

    inline int wheelRotationsToInches(int rotations)
    {
        return rotations * wheelDiameter;
    }

    inline int motorTicksToRotations(int ticks)
    {
        return ticks / ticksPerMotorRevolution;
    }

    std::vector<pros::Motor*> motors;
    pros::IMU* inertial;
    int kP, kI, kD;
    Gearsets gearset;
    int wheelDiameter; // Inches
    int wheelTrack; // Inches
    int rotation = 0;
    int gearRatio = 1;

    int feetToWheelRotations;
    int ticksPerMotorRevolution = 900;
    int maxRPM = 200;
};