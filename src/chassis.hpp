#pragma once

#include <iostream>
#include <vector>

#include "main.h"

#include "math.hpp"

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
    Chassis(std::vector<int> motorPorts, pros::IMU& inertial, float rotation = 0.0, double kP = 1.0, double kI = 0.0, double kD = 0.0, Gearsets gearset = Gearsets::GREEN, float wheelDiameter = 12.56,
        float wheelTrack = 12.0, double gearRatio = 1.0)
        : inertial(inertial), gearset(gearset), wheelDiameter(wheelDiameter), wheelTrack(wheelTrack), gearRatio(gearRatio)
    {
        for (int i = 0; i < motorPorts.size(); ++i)
        {
            if (i % 2 == 0)
                leftMotors.emplace_back(motorPorts[i]);
            else
                rightMotors.emplace_back(motorPorts[i]);
        }

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

        for (pros::Motor motor : leftMotors)
        {
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        }
         for (pros::Motor motor : rightMotors)
        {
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        }
    }

    void forward(double lengthInInches, int speed, pros::Controller& master) // Length in inches
    {
        if (speed > maxRPM)
            speed = maxRPM;

        double length = lengthInInches * (ticksPerMotorRevolution / wheelDiameter); // In motor ticks

        double currentPosition = 0.0; // In ticks
        double previousPosition = 0.0;
        double error = length - currentPosition;
        double integral = 0.0;
        double derivative = 0.0;
        double previousError = error;
        double speedPercent = speed / maxRPM;

        while (fabs(error) > 10)
        {
            currentPosition += rightMotors[0].get_position() - previousPosition;
            error = length - currentPosition;
            previousPosition = rightMotors[0].get_position();
            integral += error;
            derivative = error - previousError;

            float output = (kP * error);// + (kI * integral) + (kD * derivative) * (1);

            setMotorSpeed(error);

            previousError = error;
            master.clear();
            pros::delay(50);
            master.print(0, 0, "cp: %f", error);
            pros::delay(50);
            pros::delay(15); // 15ms dT
        }

        setMotorSpeed(0); // Stop the motor when the target position is reached
    }

    void turn(double angle, int speed)
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
        for (pros::Motor motor : leftMotors)
            motor.move(mappedSpeed * direction);
        for (pros::Motor motor : rightMotors)
            motor.move(-mappedSpeed * direction);

        while (abs(angle - fmod(inertial.get_rotation(), 360)) > 0.1)
        {
            pros::delay(5);
        }

        setMotorSpeed(0);
    }

    void traceSpline(std::vector<Point> splinePoints, int speed)
    {
        // Conversion factor from pixels to inches (12 feet = 144 inches = 1000 pixels)
        const double pixelsToInches = 144.0 / 1000.0;

        // Calculate the circumference of the wheel
        const double wheelCircumference = M_PI * wheelDiameter;

        // Iterate over all points in the spline
        for (int i = 0; i < splinePoints.size() - 1; ++i)
        {
            // Get the current point
            Point& currentPoint = splinePoints[i];

            double turnDegrees;

            if (i > 0)
                turnDegrees = currentPoint.angleToNextPoint - splinePoints[i - 1].angleToNextPoint;
            else
                turnDegrees = currentPoint.angleToNextPoint;

            if (turnDegrees > 180)
                turnDegrees -= 360;
            else if (turnDegrees < -180)
                turnDegrees += 360;

            double turnRatio = std::abs(turnDegrees) / 180.0;

            int leftSpeed = speed;
            int rightSpeed = speed;
            if (turnDegrees < 0)
                rightSpeed += turnDegrees; // Add because turnDegrees is negative
            else if (turnDegrees > 0)
                leftSpeed -= turnDegrees;

            double distance = currentPoint.distanceToNextPoint * pixelsToInches;

            double rotations = distance / wheelCircumference;

            rotations *= gearRatio;

            int degrees = static_cast<int>(rotations * 360.0);

            // Calculate the degrees to turn for each side
            int leftDegrees, rightDegrees;
            if (turnDegrees > 0) // Turning right
            {
                leftDegrees = degrees * (1 + turnRatio);
                rightDegrees = degrees * (1 - turnRatio);
            }
            else // Turning left or moving straight
            {
                leftDegrees = degrees * (1 - turnRatio);
                rightDegrees = degrees * (1 + turnRatio);
            }

            for (pros::Motor motor : leftMotors)
                motor.move_relative(leftDegrees, leftSpeed);
            for (pros::Motor motor : rightMotors)
                motor.move_relative(rightDegrees, rightSpeed);

            int motorsMoving = leftMotors.size() + rightMotors.size();

            for (pros::Motor motor : leftMotors)
            {
                if (motor.get_target_position() == motor.get_position())
                    --motorsMoving;
            }
            for (pros::Motor motor : rightMotors)
            {
                if (motor.get_target_position() == motor.get_position())
                    --motorsMoving;
            }

            while (motorsMoving > 0)
            {
                motorsMoving = leftMotors.size() + rightMotors.size();
                for (pros::Motor motor : leftMotors)
                {
                    if (motor.get_target_position() == motor.get_position())
                        --motorsMoving;
                }
                for (pros::Motor motor : rightMotors)
                {
                    if (motor.get_target_position() == motor.get_position())
                        --motorsMoving;
                }

                pros::delay(5);
            }
        }
    }


    inline void setMotorSpeed(int speed)
    {
        for (pros::Motor motor : leftMotors)
            motor.move(speed);
        for (pros::Motor motor : rightMotors)
            motor.move(speed);
    }

    inline double wheelRotationsToTicks(double rotations)
    {
        return rotations * ticksPerMotorRevolution;
    }

    inline double motorTicksToRotations(double ticks)
    {
        return ticks / ticksPerMotorRevolution;
    }

    std::vector<pros::Motor> leftMotors;
    std::vector<pros::Motor> rightMotors;
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