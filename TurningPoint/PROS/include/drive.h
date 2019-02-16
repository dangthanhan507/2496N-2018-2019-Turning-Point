#include "main.h"
#include "config.h"

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
 //get andrew's code
}
void drive_stick()
{
  //get andrew's code
}
void drive_tank()
{
  //get andrew's code
}

#endif
