#include "robot-config.h"
#include <string.h>
using namespace std;
/*
-----------------------------------
///////////////////////////////////
*/

int check() //Prints Data onto Controller Screen to see Values for auton (not necessary after auton tuning)
{
    Controller1.Screen.clearScreen();
    Gyro.startCalibration();
    BackRight.setRotation(0,rotationUnits::deg);
    BackLeft.setRotation(0,rotationUnits::deg);
    while(true)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("Gyro: %.1f",Gyro.value(rotationUnits::deg));
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print("TR: %.1f", FrontRight.rotation(rotationUnits::deg));
        Controller1.Screen.setCursor(3,0);
        Controller1.Screen.print("TL: %.1f", FrontLeft.rotation(rotationUnits::deg));
        task::sleep(20);
    }
}
/*
"""///////////////////////////"""
           Auton Code
"""///////////////////////////"""
*/
#define name_length (sizeof(auton_name)/sizeof(auton_name[0]))
string auton_name[] = {"auton1", "auton2", "auton3", "auton4", "auton5", "auton6", "auton7", "auton8"};
int auton_index = 0;
int LCD_selector()
{
    Controller1.Screen.clearScreen();
    while(true)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("Auton Name:");
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print(auton_index);
        if(Controller1.ButtonLeft.pressing() && auton_index > 0)
        {
          while(Controller1.ButtonLeft.pressing()) {}
          auton_index--;
        } 
        if(Controller1.ButtonRight.pressing() && auton_index < name_length-1)
        {
          while(Controller1.ButtonRight.pressing()) {}
          auton_index++;
        }
        task::sleep(20);
    }
}

void pre_auton( void ) //first time setup for robot
{  
    BackRight.setRotation(0,rotationUnits::deg);
    BackLeft.setRotation(0,rotationUnits::deg);
    FrontRight.setRotation(0,rotationUnits::deg);
    FrontLeft.setRotation(0,rotationUnits::deg);
    Gyro.startCalibration();
    task d(LCD_selector);
    while(Controller1.ButtonA.pressing() == false) {}
    task::stop(LCD_selector);
}

double turn_target = 0.0;
double turn_preverror = 0.0;
double turn_error = 0.0;
double turn_kp = 0.0;
double turn_ki = 0.0;
double turn_kd = 0.0; 
double turn_int = 0.0;
double turn_der = 0.0;
int value_turn = 0;
int turn()
{
   while(true)
   {
       turn_error = turn_target - Gyro.value(rotationUnits::deg);
       if(abs((int)turn_error) < 8) turn_int+=turn_error;
       else turn_int = 0;
       turn_der = turn_error - turn_preverror;
       value_turn = turn_kp*turn_error + turn_ki * turn_int + turn_kd * turn_der;
       BackLeft.spin(directionType::fwd,value_turn,velocityUnits::pct);
       FrontLeft.spin(directionType::fwd,value_turn,velocityUnits::pct);
       BackRight.spin(directionType::fwd,value_turn,velocityUnits::pct);
       BackLeft.spin(directionType::fwd,value_turn,velocityUnits::pct);
       task::sleep(15);
   }
}

void load_catapult()
{    
    while(Limit1.pressing() == false)
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    }
    CatapultLeft.stop(brakeType::coast);
    CatapultRight.stop(brakeType::coast);
}

void shoot_catapult()
{
    while(Limit1.pressing() == true)
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    }
    CatapultLeft.stop(brakeType::coast);
    CatapultRight.stop(brakeType::coast);
}

void load_balls()
{
    Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    task::sleep(1000);
    Ballintake.stop(brakeType::coast);
}

int drive_val = 90;
int error_auto = 0;
double kp_auto = 0.1;
int auto_val = 0;

int auto_straight()
{
    Gyro.startCalibration();
    while(true)
    {
         error_auto = 0 - Gyro.value(rotationUnits::deg);
         auto_val = kp_auto * error_auto;
         task::sleep(15);    
    }
}

void go_chassis(int degrees)
{
    //task t(auto_straight);
    while(FrontLeft.rotation(rotationUnits::deg) < degrees)
    {
        FrontLeft.spin(directionType::fwd,drive_val-auto_val,velocityUnits::pct);
        FrontRight.spin(directionType::fwd,drive_val+auto_val,velocityUnits::pct);
        BackLeft.spin(directionType::fwd,drive_val-auto_val,velocityUnits::pct);
        BackRight.spin(directionType::fwd,drive_val+auto_val,velocityUnits::pct);
    }
    //task::stop(auto_straight);
}
void autonomous( void ) //autonomous code that runs for 15 seconds
{
  switch(auton_index)
  {
      case 0:
      {
          go_chassis(1000);
      }
      case 1:
      {
              
      }
      case 2:
      {

      }
      case 3:
      {
              
      }
      case 4:
      {

      }
      case 5:
      {
              
      }
      case 6:
      {

      }
      case 7:
      {
              
      }
      case 8:
      {
              
      }
  }
}
/*
"""///////////////////////////"""
           Drive Code
"""///////////////////////////"""
*/
bool catapult_power = true;
bool drive_reverse = false;
int left_axis = 0;
int right_axis = 0;
void drive_tank() //tank_drive for robot and stopping with whatever brakes in the parameter.
{
    if(Controller1.ButtonB.pressing()) //Button toggles reverse driving
    {
        while(Controller1.ButtonB.pressing()) {}
        drive_reverse = !(drive_reverse);
    }
    if(drive_reverse) //checks if toggled
    {
        left_axis = -Controller1.Axis2.value();
        right_axis = -Controller1.Axis3.value();   
    }
    else //if it doesn't toggle
    {
        left_axis = Controller1.Axis3.value();
        right_axis = Controller1.Axis2.value();
    }
    //main code for tank drive using left and right joystick vertical axis
        BackLeft.spin(directionType::fwd,left_axis,velocityUnits::pct);
        FrontLeft.spin(directionType::fwd,left_axis,velocityUnits::pct);
        BackRight.spin(directionType::fwd,right_axis,velocityUnits::pct);
        FrontRight.spin(directionType::fwd,right_axis,velocityUnits::pct);
 }
void drive_arcade()
{ // code for arcade drive on robot
    left_axis = Controller1.Axis3.value()+Controller1.Axis1.value();
    right_axis = Controller1.Axis3.value()-Controller1.Axis1.value();
    BackLeft.spin(directionType::fwd,left_axis,velocityUnits::pct);
    FrontLeft.spin(directionType::fwd,left_axis,velocityUnits::pct);
    BackRight.spin(directionType::fwd,right_axis,velocityUnits::pct);
    FrontRight.spin(directionType::fwd,right_axis,velocityUnits::pct);
}
void drive_catapult() //catapult code for driving
{
    if(Controller1.ButtonR1.pressing())//hold onto A to move catapult
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);   
    }
    else if(Controller1.ButtonR2.pressing())
    {
        while(Controller1.ButtonR2.pressing()) {}
        catapult_power = !(catapult_power);
    }
    else//if not toggled
    {
        if(catapult_power)//checks if toggled
        {
            CatapultLeft.stop(brakeType::coast);
            CatapultRight.stop(brakeType::coast);
        }
        else
        {
            if(Limit1.pressing())
            {
                CatapultLeft.stop(brakeType::coast);
                CatapultRight.stop(brakeType::coast);
            }
            else
            {
                CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
                CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
            }
        }
    }
}
  
void drive_ball_intake() // drive code for ball intake
{
    if(Controller1.ButtonL1.pressing())//intake the ball if this R1 presseed
    {
        Ballintake.spin(directionType::fwd,-100,velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing())//flip cap if this R2 pressed
    {
        Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else // if no buttons pressed, stop the intake 
    {
        Ballintake.stop();
    }
}
void drive_stick_() // drive code for stick
{
    if(Controller1.ButtonA.pressing()) //if L1 pressed, spin stick clockwise
    {
        Stick.spin(directionType::fwd,50,velocityUnits::pct);
    }
    else if (Controller1.ButtonX.pressing()) //if l2 pressed, spin stick counterclockwise
    {
        Stick.spin(directionType::fwd,-50,velocityUnits::pct);
    }
    else // if no button pressed, stop stick
    {
        Stick.stop(brakeType::hold);
    }
}

void usercontrol( void ) //teleoperator code
{
  // User control code here, inside the loop
   while(true)
   {
       drive_stick_();
       drive_ball_intake();
       drive_arcade();
       drive_catapult();
   }
}

//Runs the teleop and autonomous in the robot.
int main() 
{
    pre_auton();
    task newtask(check);
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) { vex::task::sleep(100); }
}
