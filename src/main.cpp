/*
PRESETS:

Y is brake, A for matchload, B for move to matchload

Caleb: Tank Drive, Up/Down arrows to change the movement speed, R2 for intake in, L1 for intake out, R1 to launch catapult

Henry: Split arcade Drive, Up/Down arrows to change the movement speed, R1 for intake in, Left Arrow for intake out, L1 to launch catapult, Right Arrow for wings

Jack: Tank Drive, Up/Down arrows to change the movement speed, always intaking; R2 switches intake direction, L2 to launch catapult

Milo: Tank Drive, Up/Down arrows to change the movement speed, R2 for intake in, R1 for intake out, L2 to launch catapult

Jay: Split arcade Drive, Up/Down arrows to change the movement speed, Left arrow to toggle on and off intake; right is pressed to reverse intake, R2 to launch catapult

Joshua: Split arcade Drive, Up/Down arrows to change the movement speed, L2 for intake in, L1 for intake out, R2 to launch catapult
*/

// String literal to turn

#include <unordered_map>
#include <vector>
#include <array>

#include "okapi/api.hpp"
#include "api.h"

#include "drive.hpp"
#include "auton.hpp"
#include "etchaSketch.hpp"
#include "physicsEngine.hpp"
#include "main.h"

#define COMPETITION 0

pros::ADIButton limitSwitch('a');
physicsManager explosion;

double movementSpeed = 0.5;
bool jayIntakeOn = 0;

// An array of the autons
// Stores a pointer to a function that returns void and takes a pointer to a chassis as a parameter
std::array<void (*)(pros::ADIDigitalOut&), 2> autons = {DefensiveAuton, OffensiveAuton};

// Indexes of the autons
enum autonsIndex { Defensive = 0, Offensive, autonLength };

// Auton names
std::unordered_map<int, std::string> autonNames = {
	{autonsIndex::Defensive, "Defensive"},
	{autonsIndex::Offensive, "Offensive"}
};

// Default auton
autonsIndex selectedAuton = autonsIndex::Defensive;

enum driverPresets { CALEB = 0, JOSHUA, MILO, JAY, JACK, HENRY, HENRY2, TEST, length};
driverPresets selectedPreset = driverPresets::CALEB;

std::unordered_map<driverPresets, std::string> presetNames = {
	{driverPresets::CALEB, "Caleb"},
	{driverPresets::JOSHUA, "Joshua"},
	{driverPresets::HENRY, "Henry"},
	{driverPresets::HENRY2, "Henry2"},
	{driverPresets::MILO, "Milo"},
	{driverPresets::JAY, "Jay"},
	{driverPresets::JACK, "Jack"},
	{driverPresets::TEST, "Test"}
};

namespace pros
{
	constexpr int E_MAX_MOVE_SPEED = 127;
	constexpr int E_MOTOR_GEAR_200_MAX_SPEED = 200;
	constexpr int E_MOTOR_GEAR_600_MAX_SPEED = 600;
};

void autonSelection()
{
	// Prints the auton selection screen
	master.clear();
	pros::delay(100);
	master.print(0, 0, "Selecting Auton.");
	pros::delay(100);
	master.print(1, 0, "Left/Right Arrows/A");
	pros::delay(100);
	master.print(2, 0, "Auton: %s", autonNames[selectedAuton].c_str());
	pros::delay(100);

	// While 'a' hasn't been pressed
	while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
	{
		// Cycle left
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			// Subtract one from the current auton, and if it's less than 0, cycle back to the end
			selectedAuton = autonsIndex((2 + (selectedAuton - 1)) % 2);
			// Print selected auton
			master.clear_line(2);
			pros::delay(100);
			master.print(2, 0, "Auton: %s", autonNames[selectedAuton].c_str());
			pros::delay(100);
		}
		// Cycle right
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			// Add one to the current auton, and if it's greater than the length, cycle back to the beginning
			selectedAuton = autonsIndex((selectedAuton + 1) % 2);
			// Prints selected auton
			master.clear_line(2);
			pros::delay(100);
			master.print(2, 0, "Auton: %s", autonNames[selectedAuton].c_str());
			pros::delay(100);
		}
		// Stops the program from using up all the resources
		pros::delay(2);
	}
	// 'a' has been pressed
	// Print selected auton
	master.clear();
	pros::delay(100);
	master.print(0, 0, "Auton: %s", autonNames[selectedAuton].c_str());
	pros::delay(100);
	// Prompts the user to select the driver (optional), and prints the current one
	master.print(1, 0, "Select Driver");
	pros::delay(100);
	master.print(2, 0, "Driver: %s", presetNames[selectedPreset].c_str());
	pros::delay(500);
}

void presetSelection(driverPresets& selectedPreset)
{
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
		master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
	{
		// If any motors are moving
		Intake.move_velocity(0);
		Slapatapult.move_velocity(0);

		master.clear();
		pros::delay(100);
		master.print(0, 0, "Selecting Driver.");
		pros::delay(100);
		master.print(1, 0, "Left/Right Arrows/A");
		pros::delay(200);
		master.print(2, 0, "Driver: %s", presetNames[selectedPreset].c_str());
		pros::delay(100);
		
		while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
			{
				selectedPreset = driverPresets((driverPresets::length + (selectedPreset - 1)) % driverPresets::length);
				master.clear_line(2);
				pros::delay(100);
				master.print(2, 0, "Driver: %s", presetNames[selectedPreset].c_str());
				pros::delay(100);
			}
			else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
			{
				selectedPreset = driverPresets((selectedPreset + 1) % driverPresets::length);
				master.clear_line(2);
				pros::delay(100);
				master.print(2, 0, "Driver: %s", presetNames[selectedPreset].c_str());
				pros::delay(100);
			}
			pros::delay(2);
		}
		master.clear();
		pros::delay(100);
		master.print(0, 0, "Driver: %s", presetNames[selectedPreset].c_str());
		pros::delay(100);
		master.clear_line(1);
		pros::delay(100);
		master.print(1, 0, "Speed: %i%%", static_cast<int>(movementSpeed * 100));
		pros::delay(100);
		if (selectedPreset == driverPresets::HENRY2)
			inertial.tare();
	}
}

void my_task_fn(void* ignore)
{
  	while (!pros::competition::is_autonomous())
  	{
   	 	presetSelection(selectedPreset);
  		pros::delay(2);
  	}
}

// all other competition modes are blocked by this function
void initialize()
{
	inertial.reset();
	pros::lcd::initialize();
	// pros::lcd::set_background_color(0, 0, 0);
	// pros::lcd::set_text_color(0, 255, 0);
	pros::lcd::set_background_color(0, 0, 255); // Blue
	pros::lcd::set_text_color(255, 255, 255); // Peach

	pros::ADIDigitalOut pneumatics('A');
	
	chassis->setMaxVelocity(240);
	pros::delay(100);
	master.clear();
	pros::delay(200);
	master.clear();
	pros::delay(100);

#if COMPETITON
	pros::Task my_task (my_task_fn, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Driver Selection");
	autonSelection();
#endif // COMPETITION
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


void autonomous()
{
	pros::ADIDigitalOut pneumatics('A');
	while (inertial.is_calibrating())
	{
		pros::delay(2);
	}
	(autons[selectedAuton])(pneumatics); // Runs the selected auton
	//DefensiveAuton(pneumatics);
}

// HAS NOT BEEN TESTED
void headlessMode(const double movementSpeed)
{
	// Default speed
	FL.move_velocity(0);
	FR.move_velocity(0);
	ML.move_velocity(0);
	MR.move_velocity(0);
	BL.move_velocity(0);
	BR.move_velocity(0);
	
	// A value to get the left stick's degree in a range of -180 to 180
	// atan2 returns the angle in radians from -PI to PI, which then gets converted from radians to degrees
	double leftStickDegree = atan2(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * 180 / PI;
  	pros::lcd::print(0, "Left Stick Degree: %f", leftStickDegree);

	// See if the joystick has moved
	bool leftStickMoved = 0;
	if (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) || master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))
		leftStickMoved = 1;

	// Convert the robot's rotation to a value from -180 to 180
	double robotRotation = fmod(fabs(inertial.get_rotation()), 360);
	pros::lcd::print(1, "Robot rotation in 0-360: %f", robotRotation);
	robotRotation = robotRotation > 180 ? robotRotation - 360 : robotRotation;
	pros::lcd::print(2, "Robot rotation in -180-180: %f", robotRotation);


	if (leftStickMoved) // If the joystick has moved
	{
		pros::lcd::print(4, "Moved");
		// Gets the robots rotation in degrees, from -180 to 180
		// Assuming get_rotation() returns a value from 0 to 360

		
		int motorSpeed = movementSpeed * 127;
		
		double joystickDegree = leftStickDegree < 0 ? leftStickDegree + 360 : leftStickDegree; // convert joystick degree to 0-360 range for calculation
		double turnDistance = joystickDegree - robotRotation;
		pros::lcd::print(3, "Turn Distance: %f", turnDistance);
		// Converts turnDistance to a value from -180 to 180
		turnDistance = turnDistance > 180 ? turnDistance - 360 : turnDistance < -180 ? turnDistance + 360 : turnDistance;

		// If the difference is greater than 5 degrees, then rotate the robot
		if (abs(turnDistance) > 20)
		{
			// Turn the robot towards the joystick degree
			// Clockwise
			if (turnDistance > 0)
			{
				FL.move(motorSpeed);
				FR.move(-motorSpeed);
				ML.move(motorSpeed);
				MR.move(-motorSpeed);
				BL.move(motorSpeed);
				BR.move(-motorSpeed);
			}
			// Counter-Clockwise
			else
			{
				FL.move(-motorSpeed);
				FR.move(motorSpeed);
				ML.move(-motorSpeed);
				MR.move(motorSpeed);
				BL.move(-motorSpeed);
				BR.move(motorSpeed);
			}
		}
		else
		{
			// Drive forward
			FL.move(motorSpeed);
			FR.move(motorSpeed);
			ML.move(motorSpeed);
			MR.move(motorSpeed);
			BL.move(motorSpeed);
			BR.move(motorSpeed);
		}
	}
	else
	{
		pros::lcd::print(4, "Not Moved");
	}
}

void checkBrake(bool& brakeOn)
{
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
	{
		brakeOn = !brakeOn;
		if (brakeOn)
		{
			FL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			FR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			ML.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			MR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			BL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			BR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}
		else
		{
			FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			ML.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			MR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
	}
}


void driveTrain(const double movementSpeed, const driverPresets currentPreset)
{
	if (currentPreset == driverPresets::TEST)
	{
		Rotation::rotation(movementSpeed);
		pros::lcd::print(0, "Running Rotation");
	}
	// Non-Holomonic Headless Mode for Henry's second preset
	else if (currentPreset == driverPresets::HENRY)
	{
		// R2 is forward at full speed, L2 is backwards at full speed
		int leftStickY = (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) * (127 * movementSpeed)) - 
							(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) * (127 * movementSpeed));

		int rightStickX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * movementSpeed;
		FL.move(leftStickY + rightStickX);
		FR.move(leftStickY - rightStickX);

		ML.move(leftStickY + rightStickX);
		MR.move(leftStickY - rightStickX);
		
		BL.move(leftStickY + rightStickX);
		BR.move(leftStickY - rightStickX);
	}
	// Split arcade drive
	else if (currentPreset == driverPresets::HENRY || currentPreset == driverPresets::JOSHUA || currentPreset == driverPresets::JAY)
	{
		int leftStickY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * movementSpeed;
		int rightStickX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * movementSpeed;
		FL.move(leftStickY + rightStickX);
		FR.move(leftStickY - rightStickX);

		ML.move(leftStickY + rightStickX);
		MR.move(leftStickY - rightStickX);
		
		BL.move(leftStickY + rightStickX);
		BR.move(leftStickY - rightStickX);
	}
	// Tank drive
	else
	{
		int leftStickY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * movementSpeed;
		int rightStickY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * movementSpeed;
		FL.move(leftStickY);
		FR.move(rightStickY);

		ML.move(leftStickY);
		MR.move(rightStickY);
		
		BL.move(leftStickY);
		BR.move(rightStickY);
	}
}

void checkmovementSpeed(double& movementSpeed, const driverPresets currentPreset)
{
	// Everyone has the same controls, except for Caleb
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
	{
		if (!(currentPreset == driverPresets::CALEB))
    		movementSpeed = movementSpeed == 0.33 ? 0.5 : movementSpeed == 0.5 ? 1.0 : 1.0;
		else
			movementSpeed = movementSpeed <= 0.5 ? 0.75 : movementSpeed <= 0.75 ? 1.0 : 1.0;
		master.clear_line(1);
		pros::delay(100);
		master.print(1, 0, "Speed: %i%%", static_cast<int>(movementSpeed * 100));
		pros::delay(100);
	}
  	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		if (!(currentPreset == driverPresets::CALEB))
    		movementSpeed = movementSpeed == 1.0 ? 0.5 : movementSpeed == 0.5 ? 0.33 : 0.33;
		else
			movementSpeed = movementSpeed == 1.0 ? 0.75 : movementSpeed >= 0.5 ? 0.5 : 0.5;
		master.clear_line(1);
		pros::delay(100);
		master.print(1, 0, "Speed: %i%%", static_cast<int>(movementSpeed * 100));
		pros::delay(100);
	}
}

void checkIntake(const driverPresets currentPreset)
{
	Intake.move(0);
	switch (currentPreset)
	{
	// Intake for simple people
	case driverPresets::JOSHUA:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
			Intake.move(pros::E_MAX_MOVE_SPEED);
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		break;

	case driverPresets::MILO:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			Intake.move(pros::E_MAX_MOVE_SPEED);
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		break;

	case driverPresets::HENRY:
	case driverPresets::HENRY2:
		Intake.move(pros::E_MAX_MOVE_SPEED / 4);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
			Intake.move(pros::E_MAX_MOVE_SPEED);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		break;

	// Intake for complex people
	case driverPresets::JACK:
		Intake.move(pros::E_MAX_MOVE_SPEED);
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		break;

	case driverPresets::CALEB:
		Intake.move(pros::E_MAX_MOVE_SPEED / 4);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			Intake.move(pros::E_MAX_MOVE_SPEED);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		break;

	case driverPresets::JAY:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && jayIntakeOn)
			Intake.move(-pros::E_MAX_MOVE_SPEED);
		else if (jayIntakeOn)
			Intake.move(pros::E_MAX_MOVE_SPEED);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
			jayIntakeOn = !jayIntakeOn;
		break;
	}
}


/*
Don't have a catapult at the moment, but we're building it
*/
void checkCatapult(const driverPresets currentPreset)
{
	Slapatapult.move(0);
	
	switch (currentPreset)
	{
	case driverPresets::JACK:
	case driverPresets::MILO:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			Slapatapult.move(127);
		}
		break;

	case driverPresets::CALEB:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			Slapatapult.move(127);
		}
		break;

	case driverPresets::HENRY:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			Slapatapult.move(127);
		}
		break;

	case driverPresets::JOSHUA:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			Slapatapult.move(127);
		}
		break;

	case driverPresets::JAY:
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			Slapatapult.move(127);
		}
		break;
	default:
		break;
	}
}

void checkPneumatics(pros::ADIDigitalOut& pneumatics, const driverPresets currentPreset, bool& wingsOut)
{
	switch (currentPreset)
	{
	case driverPresets::JACK:
	case driverPresets::MILO:
	case driverPresets::JAY:
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
			wingsOut = !wingsOut;
		break;

	case driverPresets::JOSHUA:
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
			wingsOut = !wingsOut;
		break;
	case driverPresets::CALEB:
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
			wingsOut = !wingsOut;
		break;

	case driverPresets::HENRY:
	case driverPresets::HENRY2:
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
			wingsOut = !wingsOut;
		break;
		

	default:
		break;
	}
	if (wingsOut)
		pneumatics.set_value(1);
	else
		pneumatics.set_value(0);
}

void matchLoad()
{
	FL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	FR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	ML.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	MR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	BL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	BR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	FL.brake();
	FR.brake();
	ML.brake();
	MR.brake();
	BL.brake();
	BR.brake();

	if (distance.get() < 10)
		Slapatapult.move(127);

	while (!master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) && !master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
		&& !master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) && !master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))
	{
		pros::delay(2);
	}

	FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	ML.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	MR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void wheely()
{
	FL.move_velocity(0);
	FR.move_velocity(0);
	ML.move_velocity(0);
	MR.move_velocity(0);
	BL.move_velocity(0);
	BR.move_velocity(0);

	FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	ML.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	MR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	FL.brake();
	FR.brake();
	ML.brake();
	MR.brake();
	BL.brake();
	BR.brake();

	FL.move_relative(-1000, 600);
	FR.move_relative(-1000, 600);
	ML.move_relative(-1000, 600);
	MR.move_relative(-1000, 600);
	BL.move_relative(-1000, 600);
	BR.move_relative(-1000, 600);

	while (abs(BR.get_position() - (-1000)) > 10)
	{
    	pros::delay(10);
	}


	FL.move_relative(2000, 600);
	FR.move_relative(2000, 600);
	ML.move_relative(2000, 600);
	MR.move_relative(2000, 600);
	BL.move_relative(2000, 600);
	BR.move_relative(2000, 600);
}

void opcontrol()
{
	bool brakeOn = 0;
	pros::ADIDigitalOut pneumatics('A');
	pneumatics.set_value(0);

	master.print(0, 0, "Driver: %s", presetNames[selectedPreset].c_str());
	pros::delay(100);
	master.print(1, 0, "Speed: %i%%", static_cast<int>(movementSpeed * 100));
	pros::delay(100);

	Time::Timer time;

	bool wingsOut = 0;
	while (inertial.is_calibrating())
	{
		pros::delay(2);
	}

	while (1)
	{
#if !COMPETITION
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			if (explosion.explosion)
			{
				explosion.clearExplosion();
				explosion.updateExplosion(time);
			}
			else
			{
				explosion.explosionInit();
				explosion.explosion = 1;
			}
		}
		if (explosion.explosion)
		{
			explosion.updateExplosion(time);
		}
		if (etchaSketch())
		{
			master.clear();
			pros::delay(100);
			master.print(0, 0, "Driver: %s", presetNames[selectedPreset].c_str());
			pros::delay(100);
			master.clear_line(1);
			pros::delay(100);
			master.print(1, 0, "Speed: %i%%", static_cast<int>(movementSpeed * 100));
			pros::delay(100);
		}
		#endif // COMPETITION

		pros::lcd::print(0, "Pitch: %f", inertial.get_pitch());
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			matchLoad();
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			FR.move_relative(300, 400);
			MR.move_relative(300, 400);
			BR.move_relative(300, 400);
			pros::delay(1000);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
			wheely();

		driveTrain(movementSpeed, selectedPreset);
		checkmovementSpeed(movementSpeed, selectedPreset);
		checkCatapult(selectedPreset);
		checkIntake(selectedPreset);
		checkPneumatics(pneumatics, selectedPreset, wingsOut);
		checkBrake(brakeOn);
		
		presetSelection(selectedPreset);


		pros::delay(20);
	}
}