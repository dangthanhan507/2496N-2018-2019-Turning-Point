#include "main.h"
#include "config.h"
#include "robotLib.h"
#include "autonLib.h"
#include "PID.h"

void turn(double degrees, int ms)
{
  PID turn(150,40,830);
  double turn_target = degrees;
  double timer = pros::millis();
  double pid_value = 0;
  while(pros::millis() <= timer + ms)
  {
    pid_value = turn.Calc_POS(turn_target, gyro.get_value(),4);
    voltage_chassis(-pid_value,pid_value);
    pros::delay(15);
  }
  stop_chassis();
  pros::delay(30);
  reset_chassis();
}


void arc(double degrees, int ms, bool arc_r)
{
  PID arc(500,9,700);
  double pid_value = 0;
  double arc_target = degrees;
  double timer = pros::millis();
  while(pros::millis() <= timer + ms)
  {
    pid_value = arc.Calc_POS(arc_target, gyro.get_value(),3);
    if(arc_r) voltage_chassis(-pid_value,0);
    else voltage_chassis(0,pid_value);
    pros::delay(15);
  }
  stop_chassis();
  pros::delay(30);
  reset_chassis();
}


void forward(double encoders, int ms, int ball_intake)
{
  Ballintake.move(ball_intake);
  double auto_target = gyro.get_value();
  double fwd_target = encoders;
  double fwd_value = 0, auto_value = 0;
  double timer = pros::millis();
  PID fwd(75,0,750);
  PID auto_(75,0,0);
  while(pros::millis() <= timer + ms)
  {
      fwd_value = fwd.Calc_POS(fwd_target,BR.get_encoder_units(),3);
      auto_value = auto_.Calc_POS(auto_target,gyro.get_value(),2);
      voltage_chassis(fwd_value - auto_value, fwd_value + auto_value);
      pros::delay(15);
  }
  Ballintake.move(0);
  stop_chassis();
  pros::delay(30);
  reset_chassis();
}


void load_catapult()
{
  while(Lim_Switch.get_value() == 0)
  {
    CatL.move(127);
    CatR.move(127);
  }
  CatL.move(0);
  CatR.move(0);
}


void shoot_catapult()
{
  CatL.move(127);
  CatR.move(127);
  pros::delay(500);
  CatL.move(0);
  CatR.move(0);
}
