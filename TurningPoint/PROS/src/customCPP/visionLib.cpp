s#include "main.h"
#include "customLib/PID.h"
#include "customLib/config.h"
#include "customLib/robotLib.h"

//y increases as object goes down
pros::vision_object_s_t vision_capture(pros::vision_color_code_t COLOR_CODE)
{
  pros::vision_object_s_t object_list[3];
  vision.read_by_code(0, COLOR_CODE, 3, object_list);
  int object_index = 0;
  //largest object out there
  return object_list[0];
}

void X_AIM(pros::vision_color_code_t COLOR_CODE)
{
  while(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == true) {}
  int ang_target = 0, input = 0;
  if(COLOR_CODE = REDFLAG) ang_target = 228;
  else ang_target = 180;
  PID vis_turn(100,0,200);
  double value = 0;
  while(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == false)
  {
    input = vision_capture(COLOR_CODE).x_middle_coord;
    value = vis_turn.Calculate(ang_target,input,2);
    if(input != 0)  voltage_chassis(-value,value);
    else voltage_chassis(0,0);
    pros::delay(15);
  }
}
