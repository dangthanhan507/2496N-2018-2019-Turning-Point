#include "robot-config.h"
#include <string.h>
using namespace std;
/*
-----------------------------------
///////////////////////////////////
*/

//auton selector which uses an array of string of auton names which will be displayed on the controller lcd
#define name_length (sizeof(auton_name)/sizeof(auton_name[0]))
string auton_name[] = {"red_flag", "red_flag_park", "blue_flag", "blue_flag_park", "red_bot", "red_bot_park", "blue_bot", "blue_bot_park","skills"};
//auton_index is the index for the auton_name array and the auton pointer array of functions
int auton_index = 0;
bool catapult_power = false;
bool catapult_auto = false;
bool drive_reverse = false;
int accel_value;
//this thread checks for button toggles and switches booleans accordingly
//this is separate to make sure that toggling can happen at any time during teleop
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
        Controller1.Screen.print("Gyro: %.1f",Gyro.value(rotationUnits::deg));
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print("Accel: %d", Accelerometer.value(analogUnits::range8bit));
        Controller1.Screen.setCursor(3,0);
        Controller1.Screen.print("FR: %.1f", FrontRight.rotation(rotationUnits::deg));
        task::sleep(20);
    }
}
//this structure pid will allow for multiple pid implementations by creating a new variable defined PID 
struct PID
{
    double target = 90.0;
    double preverror = 0.0;
    double error = 0.0;
    double kp = 150;
    double ki = 40.0;
    double kd = 830; 
    double integral = 0.0;
    double der = 0.0;
    int value_l = 0;
    int value_r = 0;    
} go, turn;

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
void v_chas(int volts_left, int volts_right)
{
    BR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FL.spin(directionType::fwd,volts_left,voltageUnits::mV);
    BL.spin(directionType::fwd,volts_left,voltageUnits::mV);
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

int turn_()
{
   turn.kp = 150;
   turn.ki = 40.0;
   turn.kd = 830;
   while(true)
   {
       //instead of the motor using velocity percentages for PID, they use voltages for more accurate turns
       turn.preverror = turn.error;
       turn.error = turn.target - Gyro.value(rotationUnits::deg);
       if(abs((int)turn.error) < 4) turn.integral+=turn.error;
       else turn.integral = 0;
       turn.der = turn.error - turn.preverror;
       turn.value_r = turn.kp*turn.error + turn.ki * turn.integral + turn.kd * turn.der;
       turn.value_l = -turn.value_r;
       v_chas(turn.value_l,turn.value_r);
       task::sleep(15);
   }
}

int fwd_chas() // value created from this task will be used in the auto task
{
    go.kp = 75;
    go.ki = 0;
    go.kd = 750;
    while(true)
    {
       go.preverror = go.error;
       go.error = go.target - FrontRight.rotation(rotationUnits::deg);
       if(abs((int)go.error) < 3) go.integral+=go.error;
       else go.integral = 0;
       go.der = go.error - go.preverror;
       go.value_l = go.kp*go.error + go.ki * go.integral + go.kd * go.der;
       go.value_r = go.value_l;
       task::sleep(15);
    }
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
        auto_motor_r = go.value_r + auto_error*auto_kp;
        auto_motor_l = go.value_l - auto_error*auto_kp;
        v_chas(auto_motor_l,auto_motor_r);
        task::sleep(15);
    }
}
void v_fwd(int enc, int wait)
{
    reset_chas();
    go.target = enc;
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
    turn.target = target;
    task tur(turn_);
    task::sleep(wait);
    task::stop(turn_);
    task::sleep(200);
    reset_chas();
}
void park_robot()
{
    FrontLeft.spin(directionType::fwd,100,velocityUnits::pct);
    FrontRight.spin(directionType::fwd,100,velocityUnits::pct);
    BackLeft.spin(directionType::fwd,100,velocityUnits::pct);
    BackRight.spin(directionType::fwd,100,velocityUnits::pct);
    while(Accelerometer.value(analogUnits::range8bit) == accel_value) {} //first change of accel when front wheels get on plat
    while(Accelerometer.value(analogUnits::range8bit) != accel_value) {} //second change when the entire robot is on plat
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
    v_ball(-1000,2000,-100);
    v_fwd(1250,1400);
    v_turn(-94,1000);
    shoot_catapult();
    v_turn(-90,700);
    v_fwd(1500,1500);
    v_fwd(-200,700);
    v_turn(-45,1000);
    v_ball(-800,1000,50);
    v_turn(50,1000);
    v_fwd(-1000,1000);
    v_fwd(1000,500);
    stop_chassis();
}
void top_red_park()
{
    v_ball(-2000,2000,-100);
    v_fwd(2000,10000);
    v_turn(-90,2000);
    shoot_catapult();
    v_fwd(1500,5000);
    v_fwd(-1000,1000);
    v_turn(-45,2000);
    v_ball(-2000,2000,100);
    v_turn(-90,2000);
    park_robot();
}
void top_blue()
{
    v_ball(-2000,10000,-100);
    v_fwd(2000,10000);
    v_turn(90,2000);
    shoot_catapult();
    v_fwd(1500,5000);
    v_fwd(-1000,1000);
    v_turn(45,2000);
    v_ball(-2000,2000,100);
    v_turn(-45,2000);
    v_fwd(-1000,1000);
    v_fwd(1000,1000);
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
void bot_red()
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
    v_fwd(2000,1800);
    v_turn(180,1000);
    shoot_catapult();
    park_robot();
}
void (*autonPtr[])() = {&top_red,&top_red_park,&top_blue,&top_blue_park,&bot_red,&bot_red_park,&bot_blue,&bot_blue_park,&skills};
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
    LCD_selector();
    pre_auton();
    task vis(vision_track);
    task newtask(check);
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) { task::sleep(100); }
}
