#pragma once

#include <iostream>
#include <vector>
#include <ratio>

#include "main.h"

enum Gearsets
{
    RED = 0,
    GREEN = 1,
    BLUE = 2
};

// use average of front left and back right wheels for forward function

class Chassis
{
public:
    Chassis(std::vector<int> motorPorts, pros::IMU& inertial, int rotation = 0, double kP = 3, double kI = 0, double kD = 0, Gearsets gearset = Gearsets::GREEN, float wheelDiameter = 12.56,
        float wheelTrack = 12, double gearRatio = 1)
        : motors(motors), inertial(inertial), gearset(gearset), wheelDiameter(wheelDiameter), wheelTrack(wheelTrack), gearRatio(gearRatio)
    {
        for (int port : motorPorts)
            motors.emplace_back(port);

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

    void forward(int length, int speed, pros::Controller& master) // Length in inches
    {
        if (speed > maxRPM)
            speed = maxRPM;

        double currentPosition = 0; // In inches
        double previousPosition = 0;
        double error = length - currentPosition;
        double integral = 0;
        double derivative = 0;
        double previousError = error;
        double speedPercent = speed / maxRPM;

        while (abs(error) > 1)
        {
            previousPosition = motors[4].get_position();
            error = length - currentPosition;
            integral += error;
            derivative = error - previousError;

            float output = (kP * error) + (kI * integral) + (kD * derivative) * (speedPercent);

            for (pros::Motor motor : motors)
                motor.move(error);

            previousError = error;
            currentPosition = wheelRotationsToInches(motorTicksToRotations(motors[4].get_position() - previousPosition));
            master.clear();
            pros::delay(100);
            master.print(0, 0, "cp: %f", currentPosition);
            pros::delay(100);
            pros::delay(15); // 15ms dT
        }

        setMotorSpeed(0); // Stop the motor when the target position is reached
    }

    void turn(int angle, int speed)
    {
        if (speed > maxRPM)
            speed = maxRPM;
        // Calculate the direction to turn
        int currentAngle = fmod(inertial.get_rotation(), 360);
        int angleDifference = angle - fmod(inertial.get_rotation(), 360);
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
                motors[i].move(mappedSpeed * direction);
            else
                motors[i].move(-mappedSpeed * direction);
        }

        while (abs(angle - fmod(inertial.get_rotation(), 360)) > 0.1)
        {
            pros::delay(5);
        }

        setMotorSpeed(0);
    }

    inline void setMotorSpeed(int speed)
    {
        for (pros::Motor motor : motors)
        {
            motor.move(speed);
        }
    }

    inline double wheelRotationsToInches(double rotations)
    {
        return rotations * wheelDiameter;
    }

    inline double motorTicksToRotations(double ticks)
    {
        return ticks / ticksPerMotorRevolution;
    }

    std::vector<pros::Motor> motors;
    pros::IMU& inertial;
    double kP, kI, kD;
    Gearsets gearset;
    float wheelDiameter; // Inches
    float wheelTrack; // Inches
    float rotation = 0;
    double gearRatio = 1;

    int ticksPerMotorRevolution = 900;
    int maxRPM = 200;
};