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

        for (pros::Motor motor : motors)
        {
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        }
    }

    void forward(double length, int speed, pros::Controller& master) // Length in inches
    {
        if (speed > maxRPM)
            speed = maxRPM;

        double currentPosition = 0.0; // In inches
        double previousPosition = 0.0;
        double error = length - currentPosition;
        double integral = 0.0;
        double derivative = 0.0;
        double previousError = error;
        double speedPercent = speed / maxRPM;

        while (fabs(error) > 1)
        {
            currentPosition += wheelRotationsToInches(motorTicksToRotations(motors[1].get_position() - previousPosition));
            error = length - currentPosition;
            previousPosition = motors[1].get_position();
            integral += error;
            derivative = error - previousError;

            float output = (kP * error);// + (kI * integral) + (kD * derivative) * (1);

            for (pros::Motor motor : motors)
                motor.move(error * 1.5);

            previousError = error;
            master.clear();
            pros::delay(100);
            master.print(0, 0, "cp: %f", error);
            pros::delay(100);
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

    void traceSpline(std::vector<Point> splinePoints, int speed)
{
	// Conversion factor from pixels to inches (12 feet = 144 inches = 1000 pixels)
	const double pixelsToInches = 144.0 / 1000.0;

	// Calculate the circumference of the wheel
	const double wheelCircumference = M_PI * wheelDiameter;

	// Initialize the current direction of the robot
	Vector2 currentDirection(0, 1); // Assuming the robot starts facing right

	// Iterate over all points in the spline
	for (int i = 0; i < splinePoints.size() - 1; ++i)
	{
		// Get the current point
		Point& currentPoint = splinePoints[i];
		Point& nextPoint = splinePoints[i + 1];
		Vector2 nextDirection = Vector2(nextPoint.x, nextPoint.y) - Vector2(currentPoint.x, currentPoint.y);
		nextDirection.normalize();

		// Calculate the angle between the current direction and the next direction
		double turnDegrees = currentPoint.angleToNextPoint;

		// Turn the robot
		for (pros::Motor motor : motors)
			motor.move_relative(turnDegrees, speed);

		// Wait until the robot has finished turning
		while (motors[0].get_target_position() != motors[0].get_position() ||
			motors[1].get_target_position() != motors[1].get_position() ||
			motors[2].get_target_position() != motors[2].get_position() ||
			motors[3].get_target_position() != motors[3].get_position())
		{
			pros::delay(10);
		}

		double distance = currentPoint.distanceToNextPoint * pixelsToInches;

		// Calculate the number of rotations needed (distance / circumference)
		double rotations = distance / wheelCircumference;

		// Adjust for the gear ratio
		rotations *= gearRatio;

		// Calculate the degrees to turn (rotations * 360)
		int degrees = static_cast<int>(rotations * 360.0);

		// Move the robot
		for (pros::Motor motor : motors)
			motor.move_relative(degrees, speed);

		// TODO: Add code here to wait until the robot has finished moving
		// before continuing to the next point in the spline.
		while (motors[0].get_target_position() != motors[0].get_position() ||
			motors[1].get_target_position() != motors[1].get_position() ||
			motors[2].get_target_position() != motors[2].get_position() ||
			motors[3].get_target_position() != motors[3].get_position())
		{
			pros::delay(10);
		}

		currentDirection = nextDirection;
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