#pragma once

#include "main.h"
#include "time.hpp"

using namespace okapi::literals;

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062

pros::Motor FL(5);
pros::Motor FR(17, 1);

pros::Motor BL(4);
pros::Motor BR(19, 1);

pros::Motor Catapult(21);
pros::Motor Catapult2(11, 1);

pros::Motor Intake(18, 1); // Reversed

pros::IMU inertial(7);

pros::Controller master(pros::E_CONTROLLER_MASTER);

std::shared_ptr<okapi::OdomChassisController> chassis =
  okapi::ChassisControllerBuilder()
    .withMotors(5, 17, 19, 4)
    // Green gearset, 4 in wheel diam, 12.5 in wheel track
    .withDimensions({okapi::AbstractMotor::gearset::blue, (36.0 / 72.0)}, {{4_in, 12.5_in}, 1200L})
	.withOdometry(okapi::StateMode::CARTESIAN)
    .buildOdometry();

void turnTo(okapi::QAngle angle, const int speed)
{
	master.print(0, 0, "Angle: %f", inertial.get_rotation());
	int angleValue = angle.getValue() * (180 / PI);
	// If the desired angle is closer to the current angle on the right, turn right, otherwise turn left
	if (angleValue - inertial.get_rotation() > 0)
	{
		FL.move_velocity(speed);
		FR.move_velocity(-speed);
		BL.move_velocity(speed);
		BR.move_velocity(-speed);
	}
	else
	{
		FL.move_velocity(-speed);
		FR.move_velocity(speed);
		BL.move_velocity(-speed);
		BR.move_velocity(speed);
	}

	while (std::fabs(angleValue - inertial.get_rotation()) > 1)
	{
		pros::delay(2);
		//pros::lcd::print(0, "Angle: %f", inertial.get_rotation());
	}
	//pros::lcd::print(0, "Angle: %f", inertial.get_rotation());
	FL.brake();
	FR.brake();
	BL.brake();
	BR.brake();
}

double operator""_rotations(long double value)
{
	return static_cast<double>(value * 360);
}

double operator""_rotations(uint64_t value)
{
	return static_cast<double>(value * 360);
}

void printInertial(void* ignore)
{
	while (1)
	{
		pros::lcd::print(0, "Angle: %f", inertial.get_rotation());
		pros::delay(10);
	}

}

void ProgrammingSkills(pros::ADIDigitalOut& pneumatics)
{
	pros::Task print_inertial(printInertial);
	// Calibration
	Time::Timer timer;
	inertial.set_rotation(180);
	chassis->setState({0_tile, 0.5_tile, 180_deg});

	// Start match loading
	FR.move_relative(240, 260);
	BR.move_relative(240, 260);
	pros::delay(2000);
	Catapult.move(127);
	Catapult2.move(63.5);
	// Times how long we've been match loading
	timer.Reset();

	while (timer.ElapsedSec() <= 45)
	{
		pros::delay(2);
	}

	Catapult.move_velocity(0);
	Catapult2.move(0);

	//chassis->moveDistance(-0.4_tile);

	chassis->driveToPoint({-1.5_tile, 2_tile}, 1);
	chassis->setMaxVelocity(600);
	turnTo(0_deg, 200);
	chassis->moveDistance(2_tile);
	turnTo(180_deg, 200);

	pros::delay(200);
	pneumatics.set_value(1);
	pros::delay(200);

	chassis->moveDistanceAsync(-1.2_tile);
	pros::delay(1500);
	if (!chassis->isSettled())
	{
		chassis->stop();
	}
	chassis->moveDistance(1.2_tile);
	chassis->turnAngle(-30_deg);
	chassis->moveDistance(-1.2_tile);
}