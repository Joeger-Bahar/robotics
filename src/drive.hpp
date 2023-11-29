#pragma once

#include "main.h"
#include <memory>

#include <cmath>
#include "api.h"
#include "okapi/api.hpp"
#include "time.hpp"

//using namespace okapi;
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

Time::Timer timer;

// Make pneumatics
//pros::ADIPort pneumatics('A', pros::adi_port_config_e::E_ADI_DIGITAL_OUT);



std::shared_ptr<okapi::OdomChassisController> chassis =
  okapi::ChassisControllerBuilder()
    .withMotors(5, 17, 19, 4)
    // Green gearset, 4 in wheel diam, 12.5 in wheel track
    .withDimensions({okapi::AbstractMotor::gearset::blue, (36.0 / 72.0)}, {{4_in, 13.5_in}, 1200L})
	// .withGains(
	// 	{0.002, 0, 0.00003}, // Distance controller gains
	// 	{0.001, 0, 0.00003}, // Turn controller gains
	// 	{0.001, 0, 0.00003}  // Angle controller gains (helps drive straight)
	// )
	.withOdometry()
    .buildOdometry();

// const double liftkP = 0.001;
// const double liftkI = 0.0001;
// const double liftkD = 0.0001;

// std::shared_ptr<okapi::AsyncPositionController<double, double>> liftController = 
//   okapi::AsyncPosControllerBuilder()
//     .withMotor(3) // lift motor port 3
//     .withGains({liftkP, liftkI, liftkD})
//     .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);


namespace Rotation
{
#define TURN_MAXIMUM 63.5f

int turnBank = 0;

void rotation(const int movementSpeed)
{
	int finalTurnamount = turnBank;

	int leftStickY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * movementSpeed;
	int rightStickX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * movementSpeed;

	turnBank += rightStickX;

	if (turnBank > TURN_MAXIMUM)
		finalTurnamount = TURN_MAXIMUM;

	if (turnBank < -TURN_MAXIMUM)
		finalTurnamount = -TURN_MAXIMUM;
	
	FL.move(leftStickY + finalTurnamount);
	FR.move(leftStickY - finalTurnamount);
	
	BL.move(leftStickY + finalTurnamount);
	BR.move(leftStickY - finalTurnamount);

	turnBank -= finalTurnamount;
}

};