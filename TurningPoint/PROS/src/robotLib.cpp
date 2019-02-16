#include "main.h"
#include "robotLib.h"
#include "config.h"

void reset_chassis()
{
  BL.tare_position();
  BR.tare_position();
  TL.tare_position();
  TR.tare_position();
}

void voltage_chassis(int volt_left, int volt_right)
{
  BR.move_voltage(volt_right);
  TR.move_voltage(volt_right);
  BL.move_voltage(volt_left);
  TL.move_voltage(volt_left);
}

void stop_chassis()
{
  BR.move(0);
  BL.move(0);
  TL.move(0);
  TR.move(0);
}
