#include "robot-config.h"
#include <string.h>
using namespace std;
/*
-----------------------------------
///////////////////////////////////
*/

//auton selector which uses an array of string of auton names which will be displayed on the controller lcd
#define name_length (sizeof(auton_name)/sizeof(auton_name[0]))
string auton_name[] = {"red_flag", "red_flag_park","red_bot", "red_bot_park", "blue_flag", "blue_flag_park", "blue_bot", "blue_bot_park","skills"};
//auton_index is the index for the auton_name array and the auton pointer array of functions
int auton_index = 0;
bool catapult_power = false;
bool catapult_auto = true;
bool drive_reverse = false;
bool stick_PID = false;
int accel_value;
//this thread checks for button toggles and switches booleans accordingly
//this is separate to make sure that toggling can happen at any time during teleop
int button_toggles()
{
    while(true)
    {
        if(Controller1.ButtonB.pressing())
        {
            while(Controller1.ButtonB.pressing()) {}
            stick_PID = !stick_PID;
        }
        if(Controller1.ButtonA.pressing()) //Button toggles reverse driving
        {
            while(Controller1.ButtonA.pressing()) {}
            drive_reverse = !drive_reverse;
        }
        if(Controller1.ButtonR2.pressing())
        {
            while(Controller1.ButtonR2.pressing()) {}
            catapult_auto = !catapult_auto;
        }
    }
}

//vision_track uses a thread to print vision values on the brain LCD
int vision_track()
{
    Brain.Screen.clearScreen();
    //needs P-loop to adjust angle of robot to object
    //needs P-loop to adjust distance of robot to object
    while(true)
    {
        Vision.takeSnapshot(F_GREEN,3);
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("Object Count: %d", Vision.objectCount);
        for(int i = 1; i <= Vision.objectCount;i++)
        {
            Brain.Screen.setCursor(i+1,0);
            Brain.Screen.print("%d : Y-val: %d", i, Vision.objects[i-1].originY);
        }
        task::sleep(500);
    }
}
//check uses a thread to print different robot values onto the controller lcd. 
int check() //Prints Data onto Controller Screen to see Values for auton (not necessary after auton tuning)
{
    Controller1.Screen.clearScreen();
    Gyro.startCalibration();
    FrontRight.setRotation(0,rotationUnits::deg);
    while(true)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("AccelY: %.d",AccelerometerY.value(analogUnits::range8bit));
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print("Accel: %d", Accelerometer.value(analogUnits::range8bit));
        Controller1.Screen.setCursor(3,0);
        Controller1.Screen.print("Gyro: %.1f", Gyro.value(rotationUnits::deg));
        task::sleep(20);
    }
}
//this structure pid will allow for multiple pid implementations by creating a new variable defined PID 
class PID_
{
    private:
        double kp = 0;
        double ki = 0;
        double kd = 0;
    public:
        double preverror = 0.0;
        double error = 0.0;
        double integral = 0.0;
        double der = 0.0;        
        PID_(int P,int I, int D)
        {
            kp = P;
            ki = I;
            kd = D;
        }
        int Calc(double target, double input, int int_limit)
        {
            preverror = error;
            error = target-input;
            if(abs((int)error) < int_limit) integral += error;
            else integral = 0;
            der = error - preverror;
            return error*kp + integral*ki + der*kd;
        }
};
/*
"""///////////////////////////"""
           Auton Code
"""///////////////////////////"""
*/
//Selector code on LCD to choose auton
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
#define obj_h 5
#define obj_w 5
void vision_check()
{
    Vision.takeSnapshot(F_BLUE);
    if(Vision.largestObject.exists && Vision.largestObject.height > obj_h && Vision.largestObject.height < 7)
    {
        
    }
}
void v_chas(int volts_left, int volts_right)
{
    BR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FL.spin(directionType::fwd,volts_left,voltageUnits::mV);
    BL.spin(directionType::fwd,volts_left,voltageUnits::mV);
}
bool arc_r = false;
bool arc_l = false;
double arc_target = 0.0;
int arc_()
{
    PID_ arc(500,9,700);
    double value = 0;
    while(true)
    {
       value = arc.Calc(arc_target,Gyro.value(rotationUnits::deg),3);
       if(arc_r) v_chas(value,0);
       else if (arc_l) v_chas(0,-value);
       task::sleep(15);
    }
}
double turn_target = 0;
int turn_()
{
   PID_ turn(150,40,830);
   double value = 0;
   while(true)
   {
       value = turn.Calc(turn_target,Gyro.value(rotationUnits::deg),4);
       v_chas(value,-value);
       task::sleep(15);
   }
}
double schtick_target = 0;
int schtick() //drive code
{
    PID_ stick(0,0,0);
    while(true)
    {
        Stick.spin(directionType::fwd,stick.Calc(schtick_target,Stick.rotation(rotationUnits::deg),5),velocityUnits::pct);
        task::sleep(15);
    }
}
double go_target = 0;
double go_l = 0;
double go_r = 0;
int fwd_chas() // value created from this task will be used in the auto task
{
    PID_ fwd(75,0,750);
    while(true)
    {
       go_l = fwd.Calc(go_target, FrontRight.rotation(rotationUnits::deg),3);
       go_r = go_l;
       task::sleep(15);
    }
}

void load_catapult()
{
    while(Limit1.pressing()==false)
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    }
    CatapultLeft.stop(brakeType::hold);
    CatapultRight.stop(brakeType::hold);
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
void reset_chas()
{
    BackRight.setRotation(0,rotationUnits::deg);
    BackLeft.setRotation(0,rotationUnits::deg);
    FrontRight.setRotation(0,rotationUnits::deg);
    FrontLeft.setRotation(0,rotationUnits::deg);
}
int auto_motor_r = 0;
int auto_motor_l = 0;
double auto_kp = 100;
double auto_error = 0.0;
int auto_chas() //the value from this auto task uses fwd_chas values and will be used in slew task
{
    int auto_target = Gyro.value(rotationUnits::deg);
    while(true)
    {
        auto_error = auto_target - Gyro.value(rotationUnits::deg);
        auto_motor_r = go_r + auto_error*auto_kp;
        auto_motor_l = go_l - auto_error*auto_kp;
        v_chas(auto_motor_l,auto_motor_r);
        task::sleep(15);
    }
}
void v_fwd(int enc, int wait)
{
    reset_chas();
    go_target = enc;
    task f(fwd_chas);
    task straight(auto_chas);
    task::sleep(wait);
    task::stop(fwd_chas);
    task::stop(auto_chas);
    reset_chas();
}
void v_ball(int enc, int wait, int ball)
{
    Ballintake.spin(directionType::fwd,ball,velocityUnits::pct);
    v_fwd(enc,wait);
    Ballintake.stop(brakeType::coast);
}
void v_turn(int target, int wait)
{
    reset_chas();
    turn_target = target;
    task tur(turn_);
    task::sleep(wait);
    task::stop(turn_);
    task::sleep(50);
    reset_chas();
}
void v_arc(int target, int wait, bool r_l)
{
    arc_target = target;
    r_l == true? arc_r = true : arc_l = true;
    task tur(arc_);
    task::sleep(wait);
    task::stop(arc_);
    task::sleep(50);
    reset_chas();
    arc_r = false;
    arc_l = false;
}
void park_robot()
{
    FrontLeft.spin(directionType::fwd,-100,velocityUnits::pct);
    FrontRight.spin(directionType::fwd,-100,velocityUnits::pct);
    BackLeft.spin(directionType::fwd,-100,velocityUnits::pct);
    BackRight.spin(directionType::fwd,-100,velocityUnits::pct);
    Ballintake.spin(directionType::fwd,-100,velocityUnits::pct);
    while(AccelerometerY.value(analogUnits::range8bit) >165) {}
    task::sleep(1100);
    Ballintake.stop(brakeType::coast);
    FrontLeft.stop(brakeType::brake);
    FrontRight.stop(brakeType::brake);
    BackRight.stop(brakeType::brake);
    BackLeft.stop(brakeType::brake);
}
void stop_chassis()
{
    FrontLeft.stop(brakeType::coast);
    FrontRight.stop(brakeType::coast);
    BackRight.stop(brakeType::coast);
    BackLeft.stop(brakeType::coast);
}
void gyro_wait()
{
    Gyro.startCalibration();
    while(Gyro.isCalibrating()) {}
}
void top_red()
{
    load_catapult();
    v_ball(-990,2000,-100);
    v_fwd(1250,1400);
    v_turn(-94,1000);
    shoot_catapult();
    v_turn(-90,700);
    v_fwd(1500,1500);
    v_fwd(-200,700);
    v_turn(-45,1000);
    v_ball(-800,1000,60);
    v_turn(50,800);
    v_fwd(-1200,1000);
    v_fwd(1000,100);
    stop_chassis();
}
void top_red_park()
{
    load_catapult();
    v_ball(-1000,2000,-100);
    v_fwd(1250,1400);
    v_turn(-94,1000);
    shoot_catapult();
    v_turn(-90,700);
    v_fwd(1500,1500);
    v_fwd(-350,700);
    v_arc(0,1500,true);
    v_ball(-650,1500,100);
    v_turn(-102,1000);
    task::sleep(200);
    park_robot();
}
void bot_red()
{ // cannot flip the other cap
    v_ball(-900,2000,-100);
    v_turn(0,400);
    v_fwd(700,1000);
    v_turn(-43,1000);
    v_ball(-1200,3000,100);
    v_fwd(600,500);
    //v_ball(-1200,1200,100);
    //v_fwd(1400,1400);
    //v_turn(0,1000);
    //v_fwd(2000,2000);
    //v_turn(-90,2000);
    //shoot_catapult();
}
void bot_red_park()
{
    v_ball(-2000,2000,-100);
    v_fwd(1000,1000);
    v_turn(-45,1000);
    v_ball(-1200,1200,100);
    v_fwd(1400,1400);
    v_turn(0,1000);
    v_fwd(2000,2000);
    v_turn(-90,2000);
    shoot_catapult();
    v_fwd(1600,1500);
    v_turn(0,2000);
    park_robot();
}
void top_blue()
{
    load_catapult();
    v_ball(-1000,2000,-100);
    v_fwd(1250,1400);
    v_turn(94,1000);
    shoot_catapult();
    load_catapult();
    v_turn(90,700);
    v_fwd(1500,1500);
    v_fwd(-200,700);
    v_turn(45,1000);
    v_ball(-800,1000,60);
    v_turn(-50,800);
    v_fwd(-1200,1000);
    v_fwd(1000,100);
    stop_chassis();
}
void top_blue_park()
{
    v_ball(-2000,2000,-100);
    v_fwd(2000,10000);
    v_turn(90,2000);
    shoot_catapult();
    v_fwd(1500,5000);
    v_fwd(-1000,1000);
    v_turn(45,2000);
    v_ball(-2000,2000,100);
    v_turn(90,2000);
    park_robot();
}
void bot_blue()
{
    v_ball(-2000,2000,-100);
    v_fwd(1000,1000);
    v_turn(45,1000);
    v_ball(-1200,1200,100);
    v_fwd(1400,1400);
    v_turn(0,1000);
    v_fwd(2000,2000);
    v_turn(90,2000);
    shoot_catapult();
}
void bot_blue_park()
{
    v_ball(-2000,2000,-100);
    v_fwd(1000,1000);
    v_turn(45,1000);
    v_ball(-1200,1200,100);
    v_fwd(1400,1400);
    v_turn(0,1000);
    v_fwd(2000,2000);
    v_turn(90,2000);
    shoot_catapult();
    v_fwd(1600,1500);
    v_turn(0,2000);
    park_robot();   
} 
void skills()
{
    v_ball(-2000,2000,-100);
    v_fwd(2000,2000);
    v_turn(-91,1000);
    shoot_catapult();
    v_fwd(2000,2000);
    gyro_wait(); //gyro value resets to zero
    v_fwd(600,500);
    v_turn(45,1000);
    v_ball(-2000,2000,100);
    v_turn(135,1000);
    v_fwd(-1500,2000);
    v_fwd(1500,2000);
    v_fwd(90,500);
    v_ball(-3000,3000,100);
    v_fwd(100,500);
    v_turn(0,1000);
    v_fwd(-2000,2000);
    gyro_wait();
    v_fwd(4000,4000);
    v_turn(90,1000);
    v_ball(-2000,2000,-100);
    v_fwd(2000,2500);
    v_turn(180,1000);
    shoot_catapult();
    park_robot();
}
void (*autonPtr[])() = {&top_red,&top_red_park,&bot_red,&bot_red_park,&top_blue,&top_blue_park,&bot_blue,&bot_blue_park,&skills};
void autonomous( void ) //autonomous code that runs for 15 seconds
{
  reset_chas();
  (*autonPtr[auton_index])();
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
    if(abs(left_axis) < 3) left_axis = 0;
    if(abs(right_axis) < 3) right_axis = 0;
    //main code for tank drive using left and right joystick vertical axis
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
   LCD_selector();
   task newtask(check);
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
    task vis(vision_track);
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) { task::sleep(100); }
}
