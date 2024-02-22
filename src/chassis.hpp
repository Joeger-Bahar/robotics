#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "main.h"

#include "math.hpp"

enum Gearsets
{
    RED = 0,
    GREEN = 1,
    BLUE = 2
};

// template<class... value>
// float average(value... args)
// {
//     float sum = 0;
//     int numberOfItems = 0;
//     for (auto i : {args...})
//     {
//         sum += i;
//         ++numberOfItems;
//     }
//     return sum / numberOfItems;
// }

float average(std::vector<pros::Motor> vec1, std::vector<pros::Motor> vec2)
{
    float sum = 0.0;

    for (pros::Motor motor : vec1)
    {
        sum += motor.get_position();
    }
    for (pros::Motor motor : vec2)
    {
        sum += motor.get_position();
    }

    return sum / (vec1.size() + vec2.size());
}

// use average of front left and back right wheels for forward function

class Chassis
{
public:
    Chassis(std::vector<int> motorPorts, pros::IMU& inertial, float rotation = 0.0, double kP = 1.0, double kI = 0.0, double kD = 0.0, Gearsets gearset = Gearsets::GREEN, float wheelCircumference = 12.56,
        float wheelTrack = 12.0, double gearRatio = 1.0)
        : inertial(inertial), gearset(gearset), wheelCircumference(wheelCircumference), wheelTrack(wheelTrack), gearRatio(gearRatio)
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

        //ticksPerMotorRevolution *= gearRatio;

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
        
        // NOT USING GEAR RATIO
        speed = std::min(speed, maxRPM);


        double length = average(rightMotors, leftMotors) + (lengthInInches * ((300 / 10.21f) * gearRatio)); // In motor ticks

        master.clear();
        pros::delay(50);
        master.print(0, 0, "cp: %f", length);
        pros::delay(50);

        // for (pros::Motor motor : leftMotors)
        //     motor.tare_position();
        // for (pros::Motor motor : rightMotors)
        //     motor.tare_position();

        double currentPosition = 0.0; // In ticks
        double previousPosition = 0.0;
        double error = length - currentPosition;
        double integral = 0.0;
        double derivative = 0.0;
        double previousError = error;
        double speedPercent = speed / maxRPM;

        while (fabs(error) > 5)
        {
            currentPosition += average(rightMotors, leftMotors) - previousPosition;
            error = length - currentPosition;
            previousPosition = average(rightMotors, leftMotors);

            integral += error;
            derivative = error - previousError;

            float output = (kP * error) + (kI * integral) + (kD * derivative);

            setMotorSpeed(error * speed);

            previousError = error;
            pros::delay(15); // 15ms dT
        }

        setMotorSpeed(0); // Stop the motor when the target position is reached
    }

    void turn(double angle, int speed)
    {
        speed = std::min(speed, maxRPM);

        // Calculate the direction to turn
        float currentAngle = fmod(inertial.get_rotation(), 360);
        float targetAngle = fmod(currentAngle + angle, 360);
        float angleDifference = targetAngle - currentAngle;

        int direction = targetAngle > 180 ? -1 : 1;
        int mappedSpeed = (speed * 127 / maxRPM);

        // Turn left motors forward and right motors backward
        for (pros::Motor motor : leftMotors)
            motor.move(mappedSpeed * direction);
        for (pros::Motor motor : rightMotors)
            motor.move(-mappedSpeed * direction);

        while (fabs(targetAngle - fmod(inertial.get_rotation(), 360)) > 1.0)
        {
            pros::delay(5);
        }

        setMotorSpeed(0);
    }

    void traceSpline(std::vector<Point> splinePoints, int speed, bool backwards = 0)
    {
        speed = std::min(speed, maxRPM);

        // Conversion factor from pixels to inches (12 feet = 144 inches = 1000 pixels)
        const double pixelsToInches = 144.0 / 1000.0;
        
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
            motor.move_velocity(speed);
        for (pros::Motor motor : rightMotors)
            motor.move_velocity(speed);
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
    float wheelCircumference; // Inches
    float wheelTrack; // Inches
    float rotation = 0.0;
    double gearRatio = 1.0;

    int ticksPerMotorRevolution = 900;
    int maxRPM = 200;
};