#pragma once

#include <memory>

#include "main.h"
#include "okapi/api.hpp"
#include "api.h"

#include "units.hpp"
#include "drive.hpp"

using namespace okapi::literals;

// mm = millimeter
// cm = centimeter
// m = meter
// km = kilometer
// in = inch
// ft = foot
// yd = yard
// tile = field tile
// mi = mile
// lightyear = lightyear

// deg = degree
// rad = radians

/*
<SC7> an autonomous Win Point is awarded to any alliance that has completed the following tasks
at the end of the autonomous Period:
a. Removed the Triball from the alliance’s Match Load Zone that coincides with their Starting
Tiles. For example, in Figure 18, the red alliance must remove the Triball that begins in the
bottom-left Match Load Zone, adjacent to Robot 1’s Starting Tiles.
b. Scored at least one alliance Triball in the alliance’s own Goal.
c. Ended the autonomous Period with at least one Robot contacting their own Elevation Bar.
d. Not violated any other rules.
*/

// Need to score team triball.
// Touch elevation bar

// void turnAngle(okapi::QAngle angle)
// {
// 	int rotations[5] = {2, 16, 32, 64, 64}; // two "16"s to complete the full turn

// 	double desiredTurnDegree = inertial.get_rotation() + (angle.getValue() * (180 / PI));
// 	double currentDegree = inertial.get_rotation();
	
// 	for (int i = 0; i < 5; ++i)
// 	{
// 		chassis->turnAngle(angle / (1 / rotations[i]));

// 		int turnedAmount = currentDegree + inertial.get_rotation();

// 		if (turnedAmount >= desiredTurnDegree)
// 		{
// 			break;
// 		}
// 	}
// }
void turnTo(okapi::QAngle angle, const int speed)
{
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

	while (std::abs(angleValue - inertial.get_rotation()) > 1)
	{
		pros::delay(2);
		pros::lcd::print(0, "Angle: %f", inertial.get_rotation());
	}
	pros::lcd::print(0, "Angle: %f", inertial.get_rotation());
	FL.move_velocity(0);
	FR.move_velocity(0);
	BL.move_velocity(0);
	BR.move_velocity(0);
}


double operator""_rotations(long double value)
{
	return static_cast<double>(value * 360);
}

double operator""_rotations(uint64_t value)
{
	return static_cast<double>(value * 360);
}

void OffensiveAuton(pros::ADIDigitalOut& pneumatics)
{
	// Drive towards the goal
	chassis->moveDistance(2_tile);
	pros::delay(100);

	// Score alliance triball
	Intake.move(127);
	chassis->turnAngle(90_deg);
	pros::delay(100);
	chassis->moveDistance(-0.5_tile);
	chassis->setMaxVelocity(600);
	Intake.move(-127);
	pros::delay(350);
	Intake.move(0);
	chassis->moveDistance(0.83_tile);
	chassis->setMaxVelocity(240);
	pros::delay(100);
	chassis->moveDistance(-0.25_tile);
	pros::delay(100);

	// Return to starting position
	chassis->turnAngle(90_deg);
	pros::delay(100);
	chassis->moveDistance(2_tile);
	pros::delay(100);

	// Touch elevation bar
	chassis->turnAngle(-97_deg);
	pros::delay(100);
	chassis->moveDistance(-1.4_tile);
	pros::delay(100);
	
	master.clear();
	pros::delay(100);
}

// Score team triball
// Remove red alliance match load
// Touch elevation 


void DefensiveAuton(pros::ADIDigitalOut& pneumatics)
{
	inertial.set_rotation(-45);

	// Remove match load
	//pneumatics.set_value(1);
	pros::delay(300);
	FR.move_relative(-3_rotations, 300);
	BR.move_relative(-3_rotations, 300);
	FL.move_relative(-1.5_rotations, 150);
	BL.move_relative(-1.5_rotations, 150);

	// Finish movement
	pros::delay(500);
	//pneumatics.set_value(0);
	pros::delay(1200);

	// Hits the triballs twice
	chassis->setMaxVelocity(300);
	chassis->moveDistance(0.15_tile);
	chassis->setMaxVelocity(600);
	chassis->moveDistance(-0.65_tile);
	chassis->setMaxVelocity(300);
	chassis->moveDistance(0.65_tile);
	chassis->setMaxVelocity(600);
	chassis->moveDistance(-0.65_tile);

	// Drives away from goal and turns
	// chassis->setMaxVelocity(300);
	// chassis->moveDistance(0.25_tile);
	// turnTo(90_deg, 120);

	// // Drive towards elevation bar
	// chassis->setMaxVelocity(300);
	// chassis->moveDistance(-1.93_tile);

	// // Touch wings to evelation bar
	// pros::delay(300);
	// pneumatics.set_value(1);
	// pros::delay(300);
	// FL.move_relative(-1_rotations, 240);
	// BL.move_relative(-1_rotations, 240);
	// FR.move_relative(0.50_rotations, 120);
	// BR.move_relative(0.50_rotations, 120);
}