#pragma once

#include <unordered_map>
#include <string>

#include "drive.hpp"
#include "main.h"

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 272

int x = SCREEN_WIDTH / 2, y = SCREEN_HEIGHT / 2;
int additionalThickness = 0;

int prevX = x, prevY = y;


int64_t colors[8] = { COLOR_WHITE, COLOR_RED, COLOR_YELLOW, COLOR_PINK, COLOR_GREEN, COLOR_BROWN, COLOR_BLUE, COLOR_PURPLE };

int currentColorIndex = 0;

std::unordered_map<int64_t, std::string> colorNames = {
	{colors[0], "White"},
	{colors[1], "Red"},
	{colors[2], "Yellow"},
	{colors[3], "Pink"},
	{colors[4], "Green"},
	{colors[5], "Brown"},
	{colors[6], "Blue"},
    {colors[7], "Purple"}
};

bool etchaSketch()
{
    bool returnValue = 0;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        Intake.move(0);
        Slapatapult.move(0);

        returnValue = 1;
        master.clear();
        pros::delay(100);
        master.print(0, 0, "EtchaSketch On");
        pros::delay(100);
        master.print(1, 0, "Color: %s", colorNames[colors[currentColorIndex]].c_str());
        pros::delay(100);
        master.print(2, 0, "Thickness: %i", additionalThickness + 1);
        pros::delay(100);

        while (1)
        {
            // Exit
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
            {
                master.clear_line(2);
                pros::delay(100);
                break;
            }

            // Adjusting thickness and color
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
            {
                currentColorIndex = (currentColorIndex + 1) % 8;

                master.clear_line(1);
                pros::delay(100);
                master.print(1, 0, "Color: %s", colorNames[colors[currentColorIndex]].c_str());
                pros::delay(100);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
            {
                currentColorIndex = (8 + (currentColorIndex - 1)) % 8;

                master.clear_line(1);
                pros::delay(100);
                master.print(1, 0, "Color: %s", colorNames[colors[currentColorIndex]].c_str());
                pros::delay(100);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
            {
                ++additionalThickness;
                master.clear_line(2);
                pros::delay(100);
                master.print(2, 0, "Thickness: %i", additionalThickness + 1);
                pros::delay(100);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
            {
                if (additionalThickness > 0)
                {
                    --additionalThickness;
                    master.clear_line(2);
                    pros::delay(100);
                    master.print(2, 0, "Thickness: %i", additionalThickness + 1);
                    pros::delay(100);
                }
            }

            int rightStickX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            int leftStickY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

            // Adds or subtracts 1 depending on if the values are signed or not
            if (rightStickX && x <= SCREEN_WIDTH && x >= 0)
                x += rightStickX / abs(rightStickX);
            if (leftStickY && y <= SCREEN_HEIGHT && y >= 0)
                y -= leftStickY / abs(leftStickY); // y-value is upside down


            pros::screen::set_pen(colors[currentColorIndex]);
            pros::screen::fill_rect(x, y, x + additionalThickness, y + additionalThickness);

            // Clear Screen
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) 
                && master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
                pros::screen::erase();

            prevX = x;
            prevY = y;
            pros::delay(20);
        }
    }
    pros::screen::erase();
    return returnValue;
}