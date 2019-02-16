#include "main.h"
#include "config.h"
#include <string>
using namespace pros;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
#define AUTON_LENGTH 7
void LCD_SELECTION()
{
	std::string auton[] = {"R_FLAG_TOP", "R_PARK_TOP", "R_FLAG_BOT", "R_PARK_BOT",
	"B_FLAG_TOP", "B_PARK_TOP", "B_FLAG_BOT", "B_FLAG_BOT"};
	controller.clear();
	while(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)==false)
	{
		controller.set_text(0,0, auton[auton_index].c_str());
		if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)
	 && auton_index < 7)
			auton_index++;
		else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)
	 && auton_index > 0)
			auton_index--;
		delay(20);
	}
}

void initialize()
{
	gyro.reset();
	BL.tare_position();
	BR.tare_position();
	TL.tare_position();
	TR.tare_position();

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
void competition_initialize() {LCD_SELECTION();}
