#include "main.h"
#include "config.h"
#include "visionLib.h"
#ifndef DRIVE_H
#define DRIVE_H


bool catapult_auto = true;
bool catapult_load = true;

void drive_catapult()
{
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) catapult_auto = !catapult_auto;
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
  {
    CatL.move(127);
    CatR.move(127);
    catapult_load = true;
  }
  else
  {
    if(catapult_auto && catapult_load)
    {
      CatL.move(127);
      CatR.move(127);
      if(Lim_Switch.get_value() == 1) catapult_load = false;
    }
    else
    {
      CatL.move(0);
      CatR.move(0);
    }
  }
}
void drive_ball_intake()
{
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) Ballintake.move(-100);
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) Ballintake.move(100);
  else Ballintake.move(0);
}
void drive_stick()
{
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) Stick.move(63);
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) Stick.move(-63);
  else Stick.move(0);
}

int drive_l = 0, drive_r = 0, side_axis = 0;
int left_axis = 0, right_axis = 0;
bool reverse_chassis = false;

void drive_tank()
{
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) reverse_chassis = !reverse_chassis;
  if(reverse_chassis)
  {
    left_axis = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    right_axis = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    side_axis = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  }
  else
  {
    left_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    right_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    side_axis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  }
  if((side_axis > 90 || side_axis < -90) && (left_axis < 80 && left_axis > -80))
  {
    drive_r = -side_axis/3;
    drive_l = side_axis/3;
  }
  else
  {
    drive_l = left_axis;
    drive_r = right_axis;
  }
  BR.move(drive_r);
  TR.move(drive_r);
  BL.move(drive_l);
  TL.move(drive_l);
}

void drive_vision()
{
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) X_AIM(REDFLAG);
}
#endif
