#include "robot-config.h"
#include <string.h>
using namespace std;
/*
-----------------------------------
///////////////////////////////////
*/
#define name_length (sizeof(auton_name)/sizeof(auton_name[0]))
string auton_name[] = {"red_flag", "red_flag_park", "blue_flag", "blue_flag_park", "red_bot", "red_bot_park", "blue_bot", "blue_bot_park","skills"};
int auton_index = 0;
bool catapult_power = false;
bool catapult_auto = false;
bool drive_reverse = false;
int accel_value;
int button_toggles()
{
    while(true)
    {
        if(Controller1.ButtonB.pressing()) //Button toggles reverse driving
        {
            while(Controller1.ButtonB.pressing()) {}
            drive_reverse = !drive_reverse;
        }
        if(Controller1.ButtonR2.pressing())
        {
            while(Controller1.ButtonR2.pressing()) {}
            catapult_auto = !catapult_auto;
        }
    }
}
int vision_track()
{
    Brain.Screen.clearScreen();
    //needs P-loop to adjust angle of robot to object
    //needs P-loop to adjust distance of robot to object
    //needs a function to eliminate all of the objects but one which is the highest flag it detects
    //vision sensor snapshot detects an array of objects with a matrix of values that can be called
    while(true)
    {
        Vision.takeSnapshot(F_GREEN,3);
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("CenterX: %d", Vision.largestObject.centerX);
        Brain.Screen.setCursor(2,0);
        Brain.Screen.print("CenterY: %d", Vision.largestObject.centerY);
        Brain.Screen.setCursor(3,0);
        Brain.Screen.print("Count: %d", Vision.objectCount);
        Brain.Screen.setCursor(4,0);
        Brain.Screen.print("Length: %d", Vision.largestObject.width);
        Brain.Screen.setCursor(5,0);
        Brain.Screen.print("Width: %d", Vision.largestObject.height);
        Brain.Screen.setCursor(6,0);
        Brain.Screen.print("Does Exist?: %b", Vision.largestObject.exists);
        Brain.Screen.setCursor(7,0);
        Brain.Screen.print("Width: %d", Vision.largestObject.id);
        
        task::sleep(15);
    }
}
int check() //Prints Data onto Controller Screen to see Values for auton (not necessary after auton tuning)
{
    Controller1.Screen.clearScreen();
    Gyro.startCalibration();
    FrontRight.setRotation(0,rotationUnits::deg);
    while(true)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("Gyro: %.1f",Gyro.value(rotationUnits::deg));
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print("Accel: %d", Accelerometer.value(analogUnits::range8bit));
        Controller1.Screen.setCursor(3,0);
        Controller1.Screen.print(auton_name[auton_index].c_str());
        task::sleep(20);
    }
}
/*
"""///////////////////////////"""
           Auton Code
"""///////////////////////////"""
*/
void LCD_selector()
{
    Controller1.Screen.clearScreen();
    while(Controller1.ButtonUp.pressing() == false)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("Auton Name:");
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print(auton_name[auton_index].c_str());
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
    accel_value = Accelerometer.value(analogUnits::range8bit);
    BackRight.setRotation(0,rotationUnits::deg);
    BackLeft.setRotation(0,rotationUnits::deg);
    FrontRight.setRotation(0,rotationUnits::deg);
    FrontLeft.setRotation(0,rotationUnits::deg);
    CatapultLeft.setRotation(0,rotationUnits::deg);
    CatapultRight.setRotation(0,rotationUnits::deg);
    Stick.setRotation(0,rotationUnits::deg);
    Ballintake.setRotation(0,rotationUnits::deg);
    Gyro.startCalibration();
}

double turn_target = 90.0;
double turn_preverror = 0.0;
double turn_error = 0.0;
double turn_kp = 150;
double turn_ki = 40.0;
double turn_kd = 830; 
double turn_int = 0.0;
double turn_der = 0.0;
int value_turn = 0;
int turn()
{
   while(true)
   {
       //instead of the motor using velocity percentages for PID, they use voltages for more accurate turns
       turn_preverror = turn_error;
       turn_error = turn_target - Gyro.value(rotationUnits::deg);
       if(abs((int)turn_error) < 4) turn_int+=turn_error;
       else turn_int = 0;
       turn_der = turn_error - turn_preverror;
       value_turn = turn_kp*turn_error + turn_ki * turn_int + turn_kd * turn_der;
       BR.spin(directionType::fwd,value_turn,voltageUnits::mV);
       BL.spin(directionType::fwd,-value_turn,voltageUnits::mV);
       FR.spin(directionType::fwd,value_turn,voltageUnits::mV);
       FL.spin(directionType::fwd,-value_turn,voltageUnits::mV);
       task::sleep(15);
   }
}
#define obj_h 5
#define obj_w 5
void vision_check()
{
    Vision.takeSnapshot(F_BLUE);
    if(Vision.largestObject.exists && Vision.largestObject.height > obj_h && Vision.largestObject.height < 7)
    {
        
    }
}

void load_catapult()
{
    while(Limit1.pressing()==false)
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    }
    CatapultLeft.stop(brakeType::coast);
    CatapultRight.stop(brakeType::coast);
}
void shoot_catapult()
{
    CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
    CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    task::sleep(500);
    CatapultLeft.stop(brakeType::coast);
    CatapultRight.stop(brakeType::coast);
    task::sleep(100);
}

void load_balls()
{
    Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    task::sleep(1000);
    Ballintake.stop(brakeType::coast);
}
void reset_chas()
{
    BackRight.setRotation(0,rotationUnits::deg);
    BackLeft.setRotation(0,rotationUnits::deg);
    FrontRight.setRotation(0,rotationUnits::deg);
    FrontLeft.setRotation(0,rotationUnits::deg);
}

void go_chassis(int degrees, int speed)
{
    while(abs((int)FrontRight.rotation(rotationUnits::deg)) < abs(degrees))
    {
        FrontLeft.spin(directionType::fwd,speed,velocityUnits::pct);
        FrontRight.spin(directionType::fwd,speed,velocityUnits::pct);
        BackLeft.spin(directionType::fwd,speed,velocityUnits::pct);
        BackRight.spin(directionType::fwd,speed,velocityUnits::pct);
    }
    FrontLeft.stop(brakeType::brake);
    FrontRight.stop(brakeType::brake);
    BackLeft.stop(brakeType::brake);
    BackRight.stop(brakeType::brake);
    task::sleep(200);
    reset_chas();
}

void stop_all()
{
    FrontLeft.stop(vex::brakeType::coast);
    FrontRight.stop(vex::brakeType::coast);
    BackLeft.stop(vex::brakeType::coast);
    BackRight.stop(vex::brakeType::coast);
    CatapultLeft.stop(vex::brakeType::coast);
    CatapultRight.stop(vex::brakeType::coast);
    Ballintake.stop(vex::brakeType::coast);
    Stick.stop(vex::brakeType::coast);
}
void park_robot()
{
    //code must be used with accelerometer to finish 
    //use the z axis 
    FrontLeft.spin(directionType::fwd,100,velocityUnits::pct);
    FrontRight.spin(directionType::fwd,100,velocityUnits::pct);
    BackLeft.spin(directionType::fwd,100,velocityUnits::pct);
    BackRight.spin(directionType::fwd,100,velocityUnits::pct);
    while(Accelerometer.value(analogUnits::range8bit) >accel_value-6) {} //first change of accel when front wheels get on plat
    while(Accelerometer.value(analogUnits::range8bit) < accel_value-4) {} //second change when the entire robot is on plat
    FrontLeft.stop(brakeType::brake);
    FrontRight.stop(brakeType::brake);
    BackRight.stop(brakeType::brake);
    BackLeft.stop(brakeType::brake);
}
void ball_chassis(int enc_chas,int sp_chas, int wait, int sp_ball)
{
    Ballintake.spin(directionType::fwd,sp_ball,velocityUnits::pct);
    go_chassis(enc_chas,sp_chas);
    task::sleep(wait);
    Ballintake.stop(brakeType::coast);    
}
void turn_aut(int target, int wait)
{
    turn_target = target;
    task tur(turn);
    task::sleep(wait);
    task::stop(turn);
    task::sleep(200);
    reset_chas();
}
void autonomous( void ) //autonomous code that runs for 15 seconds
{
  reset_chas();
  switch(auton_index)
  {
      case 0: //top red square no park
      {
          //load_catapult();
          ball_chassis(-900,-65,1500,-100);
          //go_chassis(1150,90);
          //turn_aut(-90,2000);
          //shoot_catapult();
          //go_chassis(1400,90);
          //go_chassis(-100,-90);
          //turn_aut(-15,600);
          //go_chassis(-200,90);
          //turn_aut(-17,500);
          //ball_chassis(-600,-90,1000,100);
          //load_catapult();
          //turn_aut(45,500);
          //go_chassis(-500,-90);
          //go_chassis(500,90);
          break;
      }
      case 1: //top red square park
      {
          load_catapult();
          ball_chassis(-1200,-65,1500,-100);
          go_chassis(1100,90);
          turn_aut(-98,2000);
          go_chassis(-100,90);
          shoot_catapult();
          go_chassis(1400,90);
          go_chassis(-100,-90);
          turn_aut(-15,600);
          go_chassis(-200,90);
          turn_aut(-17,500);
          ball_chassis(-600,-90,1000,100);
          load_catapult();
          //An's untuned autonomous extension
          turn_aut(45,500);
          go_chassis(-500,-90);
          go_chassis(500,90);
          //An's parking auton
          turn_aut(-90,600);
          //park_robot();
          break;              
      }
      case 2: //top blue square no park
      {
          load_catapult();
          ball_chassis(-1200,-65,1500,-100);
          go_chassis(1100,90);
          turn_aut(98,2000);
          go_chassis(-100,90);
          shoot_catapult();
          go_chassis(1400,90);
          go_chassis(-100,-90);
          turn_aut(15,600);
          go_chassis(-200,90);
          turn_aut(17,500);
          ball_chassis(-600,-90,1000,100);
          load_catapult();
          turn_aut(-45,500);
          go_chassis(-500,-90);
          go_chassis(500,90);
          turn_aut(90,600);
          break;
      }
      case 3: //top blue square park
      {
          load_catapult();
          ball_chassis(-1200,-65,1500,-100);
          go_chassis(1100,90);
          turn_aut(98,2000);
          go_chassis(-100,90);
          shoot_catapult();
          go_chassis(1400,90);
          go_chassis(-100,-90);
          turn_aut(15,600);
          go_chassis(-200,90);
          turn_aut(17,500);
          ball_chassis(-600,-90,1000,100);
          turn_aut(-45,500);
          go_chassis(-500,-90);
          go_chassis(500,90);
          turn_aut(90,600);
          park_robot();
          break;
      }
      case 4: //bot red square no park
      {
          load_catapult();
          ball_chassis(-2000,-60,1500,-100);
          go_chassis(2000,90);
          turn_aut(-98,2000);
          shoot_catapult();
          turn_aut(0,2000);
          go_chassis(-1000,-90);
          turn_aut(45,1000);
          ball_chassis(-600,-90,1000,100);
          go_chassis(600,90);
          turn_aut(-90,3000);
          break;
      }
      case 5: //bot red square park
      {
          load_catapult();
          ball_chassis(-2000,-60,1500,-100);
          go_chassis(2000,90);
          turn_aut(-98,2000);
          shoot_catapult();
          turn_aut(0,2000);
          go_chassis(-1000,-90);
          turn_aut(45,1000);
          ball_chassis(-600,-90,1000,100);
          go_chassis(600,90);
          turn_aut(-90,3000);
          park_robot();
          break;
      }
      case 6: //bot blue square no park
      {
          load_catapult();
          ball_chassis(-2000,-60,1500,-100);
          go_chassis(2000,90);
          turn_aut(98,2000);
          shoot_catapult();
          turn_aut(0,2000);
          go_chassis(-1000,-90);
          turn_aut(-45,1000);
          ball_chassis(-600,-90,1000,100);
          go_chassis(600,90);
          turn_aut(90,3000);
          break;          
      }
      case 7: //bot blue square park
      {
          load_catapult();
          ball_chassis(-2000,-60,1500,-100);
          go_chassis(2000,90);
          turn_aut(98,2000);
          shoot_catapult();
          turn_aut(0,2000);
          go_chassis(-1000,-90);
          turn_aut(-45,1000);
          ball_chassis(-600,-90,1000,100);
          go_chassis(600,90);
          turn_aut(90,3000);
          park_robot();
          break;
      }
      case 8: //skills
      {
          break;
      }
  }
}
/*
"""///////////////////////////"""
           Drive Code
"""///////////////////////////"""
*/
int left_axis = 0;
int right_axis = 0;
void drive_tank() //tank_drive for robot and stopping with whatever brakes in the parameter.
{

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
void drive_catapult()
{
    if(catapult_auto)
    {
        if(catapult_power)
        {
            CatapultLeft.stop(brakeType::coast);
            CatapultRight.stop(brakeType::coast);
        }
        if(Controller1.ButtonR1.pressing())
        {
            catapult_power = false;
            CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
            CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
        }
        else
        {
            if(Limit1.pressing())
            {
                CatapultLeft.stop(brakeType::coast);
                CatapultRight.stop(brakeType::coast);
                catapult_power = true;
            }
            else if(catapult_power == false)
            {
                CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
                CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
            }
        }
    }
    else
    {
        if(Controller1.ButtonR1.pressing())
        {
            CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
            CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
        }
        else
        {
            CatapultLeft.stop(brakeType::coast);
            CatapultRight.stop(brakeType::coast);
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
   task b(button_toggles);
   while(true)
   {
       drive_catapult();
       drive_stick_();
       drive_ball_intake();
       drive_tank();
   }
}
//Runs the teleop and autonomous in the robot.
int main() 
{
    Vision.setWifiMode(vision::wifiMode::off);
    pre_auton();
    LCD_selector();
    task vis(vision_track);
    task newtask(check);
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) { task::sleep(100); }
}
