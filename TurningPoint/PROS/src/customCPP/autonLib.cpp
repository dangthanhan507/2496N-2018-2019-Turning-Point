#include "main.h"
#include "customLib/config.h"
#include "customLib/robotLib.h"
#include "customLib/autonLib.h"
#include "customLib/PID.h"

void turn(double degrees, int ms)//POSITION PID
{
  PID turn(150,40,830);
  double turn_target = degrees;
  double timer = pros::millis();
  double pid_value = 0;
  while(pros::millis() <= timer + ms)
  {
    pid_value = turn.Calculate(turn_target, gyro.get_value(),4);
    voltage_chassis(-pid_value,pid_value);
    pros::delay(15);
  }
  stop_chassis();
  pros::delay(30);
  reset_chassis();
}


void arc(double degrees, int ms, bool arc_r) //POSITION PID
{
  PID arc(500,9,700);
  double pid_value = 0;
  double arc_target = degrees;
  double timer = pros::millis();
  while(pros::millis() <= timer + ms)
  {
    pid_value = arc.Calculate(arc_target, gyro.get_value(),3);
    if(arc_r) voltage_chassis(-pid_value,0);
    else voltage_chassis(0,pid_value);
    pros::delay(15);
  }
  stop_chassis();
  pros::delay(30);
  reset_chassis();
}


void forward(double encoders, int ms, int ball_intake) //POSITION PID
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
      fwd_value = fwd.Calculate(fwd_target,BR.get_encoder_units(),3);
      if(fwd_value >= 11500) fwd_value = 11500;
      if(fwd_value <= -11500) fwd_value = -11500;
      auto_value = auto_.Calculate(auto_target,gyro.get_value(),2);
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
