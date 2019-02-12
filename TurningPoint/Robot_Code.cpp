#include "robot-config.h"
#include <string.h>
using namespace std;
//auton selector which uses an array of string of auton names which will be displayed on the controller lcd
#define name_length (sizeof(auton_name)/sizeof(auton_name[0]))
string auton_name[] = {"red_flag", "red_flag_park","red_bot", "red_bot_park", "blue_flag", "blue_flag_park", "blue_bot", "blue_bot_park","skills"};
//auton_index is the index for the auton_name array and the auton pointer array of functions
int auton_index = 0;

int check() //Prints Data onto Controller Screen to see Values for auton (not necessary after auton tuning)
{
    Controller1.Screen.clearScreen();
    Gyro.startCalibration();
    FrontRight.setRotation(0,rotationUnits::deg);
    while(true)
    {
        Controller1.Screen.setCursor(1,0);
        Controller1.Screen.print("Temp: %.1f", FrontLeft.temperature(percentUnits::pct));
        Controller1.Screen.setCursor(2,0);
        Controller1.Screen.print("Gyro: %.1f", Gyro.value(rotationUnits::deg));
        Controller1.Screen.setCursor(3,0);
        //Controller1.Screen.print("Temp: %.2f", BackRight.temperature(vex::percentUnits::pct));
        Controller1.Screen.print("accelY: %.d", AccelerometerY.value(analogUnits::range8bit));
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

//Auton Code
//Selector code on LCD to choose auton

bool red1 = true;
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
        if(auton_index <= 3) red1 = true;
        else red1 = false;
    }
}

void pre_auton( void ) //first time setup for robot
{
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
double arc_target = 0.0, turn_target = 0.0, go_target = 0.0, ang_target = 0.0;

void v_chas(int volts_left, int volts_right)
{
    BR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FR.spin(directionType::fwd,volts_right,voltageUnits::mV);
    FL.spin(directionType::fwd,volts_left,voltageUnits::mV);
    BL.spin(directionType::fwd,volts_left,voltageUnits::mV);
}
bool arc_r = false;
bool arc_l = false;
int arc_()
{
    PID_ arc(500,9,700);
    double value = 0;
    while(true)
    {
       value = arc.Calc(arc_target,Gyro.value(rotationUnits::deg),3);
       if(arc_r) v_chas(-value,0);
       else if (arc_l) v_chas(0,value);
       task::sleep(15);
    }
}
int turn_()
{
   PID_ turn(150,40,830);
   double value = 0;
   while(true)
   {
       value = turn.Calc(turn_target,Gyro.value(rotationUnits::deg),4);
       v_chas(-value,value);
       task::sleep(15);
   }
}
double go_l = 0;
double go_r = 0;

int fwd_chas() // value created from this task will be used in the auto task
{
    PID_ fwd(75,0,750);
    while(true)
    {
       go_l = fwd.Calc(go_target, BackRight.rotation(rotationUnits::deg),3);
       go_r = go_l;
       task::sleep(15);
    }
}
void stop_chassis()
{
    FrontLeft.stop(brakeType::coast);
    FrontRight.stop(brakeType::coast);
    BackRight.stop(brakeType::coast);
    BackLeft.stop(brakeType::coast);
}

#define HEIGHT_ 140
void vision_capture(int& theta_val)
{
    int color_height=0, colorX = 0;
    if(red) Vision.takeSnapshot(BLUEBOI,2);
    else Vision.takeSnapshot(REDBOI,2);
    if (Vision.objects[0].height < HEIGHT_)
    {
        color_height = Vision.objects[0].centerY;
        colorX = Vision.objects[0].centerX;
    }
    else if(Vision.objects[1].exists)
    {
        color_height = Vision.objects[1].centerY;
        colorX = Vision.objects[1].centerX;
    }
    Vision.takeSnapshot(GREENBOI,8);
    int find = 1000;
    int compar = 0;
    int idx = 0;
    for(int i= 0; i < Vision.objectCount;i++)
    {
        compar = abs(Vision.objects[i].centerY - color_height) + abs(Vision.objects[i].centerX - colorX);
        if(compar < find)
        {
            find = compar;
            idx = i;
        }
    }
    theta_val = Vision.objects[idx].centerX;
}
int vis_theta()
{
    if(red1) ang_target = 180;
    else ang_target = 228;
    int input = 0;
    PID_ vis_turn(100,0,200);
    double value = 0;
    while(true)
    {
        vision_capture(input);
        value = vis_turn.Calc(ang_target,input,2);
        v_chas(-value,value);
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
    stop_chassis();
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
    stop_chassis();
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
void shoot_catapult()
{
    CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
    CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);
    task::sleep(500);
    CatapultLeft.stop(brakeType::coast);
    CatapultRight.stop(brakeType::coast);
    task::sleep(50);
}
void gyro_wait()
{
    Gyro.startCalibration();
    while(Gyro.isCalibrating()) {}
}
void top_red()
{//gud
    v_ball(-1050,2000,-100);
    v_turn(0,300);
    v_fwd(1100,1200);
    v_turn(-97,1000);
    shoot_catapult();
    v_turn(-89,1000);
    v_fwd(1500,1200);
    v_fwd(-400,700);
    Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    v_arc(0,1500,true);
    v_ball(-680,1200,100);
    v_turn(65,800);
    v_fwd(-1300,1000);
    v_fwd(1000,300);
    stop_chassis();
}
void top_red_park()
{   //gud
    v_ball(-1050,2000,-100);
    v_turn(0,300);
    v_fwd(1100,1200);
    v_turn(-97,1000);
    shoot_catapult();
    v_turn(-89,1000);
    v_fwd(1500,1200);
    v_fwd(-400,700);
    Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    v_arc(0,1000,true);
    v_ball(-710,1200,100);
    v_turn(-94,1300);
    v_ball(-1900,2400,-100);
}
void bot_red()
{ //gud
    v_ball(-920,2000,-100);
    v_turn(0,400);
    v_fwd(120,1000);
    v_turn(-95,1500);
    v_fwd(-200,900);
    shoot_catapult();
    v_turn(95, 1700);
    task::sleep(300);
}
void bot_red_park()
{
    v_ball(-920,2000,-100);
    v_turn(0,400);
    v_fwd(120,1000);
    v_turn(-95,1500);
    v_fwd(-200,900);
    shoot_catapult();
    v_turn(95, 1700);
    task::sleep(300);
}
void top_blue()
{//gud
    v_ball(-1050,2000,-100);
    v_turn(0,300);
    v_fwd(1100,1200);
    v_turn(101,1000);
    shoot_catapult();
    v_turn(95,900);
    v_fwd(1500,1500);
    v_fwd(-200,700);
    v_turn(43,1000);
    v_ball(-800,1000,70);
    v_turn(-50,800);
    v_fwd(-1250,1000);
    v_fwd(1000,100);
    stop_chassis();

}
void top_blue_park()
{
    v_ball(-1050,2000,-100);
    v_turn(0,400);
    v_fwd(1100,1400);
    v_turn(101,1000);
    shoot_catapult();
    v_turn(95,900);
    v_fwd(1500,1200);
    v_fwd(-200,500);
    Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    v_arc(0,1000,false);
    v_ball(-650, 1000, 100);
    v_turn(98, 900);
    v_ball(-1900,2400,-100);
}
void bot_blue()
{
    v_ball(-920,2000,-100);
    v_turn(0,400);
    v_fwd(120,1000);
    v_turn(95,1500);
    v_fwd(-200,900);
    shoot_catapult();
    v_turn(-95, 1700);
}
void bot_blue_park()
{
    v_ball(-920,2000,-100);
    v_turn(0,400);
    v_fwd(120,1000);
    v_turn(95,1500);
    v_fwd(-200,900);
    shoot_catapult();
    v_turn(-95, 1700);
    task::sleep(300);
}
void skills()
{
    v_ball(-950,2000,-100);
    v_turn(0,300);
    v_fwd(1250,1200);
    v_turn(-95,1000);
    shoot_catapult();
    v_fwd(1500,1200);
    v_fwd(-200,700);
    v_arc(0,1500,true);
    v_ball(-680,1200,80);
    v_turn(-94,1300);
    v_fwd(-1800,2400);
    v_turn(0,1000);
    v_fwd(-800,2000);
    /*v_ball(-1200,2000,-100);
    v_fwd(1430,1500);
    v_turn(-101,1000);
    shoot_catapult();
    load_catapult();
    v_turn(-87, 300);
    v_fwd(1400,1300);
    v_fwd(-200,700);*/
}
void (*autonPtr[])() = {&top_red,&top_red_park,&bot_red,&bot_red_park,&top_blue,&top_blue_park,&bot_blue, &bot_blue_park, &skills};
void autonomous( void ) //autonomous code that runs for 15 seconds
{
    reset_chas();
    (*autonPtr[auton_index])();
}
//Drive Code
int left_axis = 0;
int right_axis = 0;
int side_axis = 0;
bool re_flag = true;
bool start_aim = false;
bool catapult_power = false;
bool re_catapult_auto = true;
bool re_drive_reverse = false;
bool catapult_auto = true;
bool drive_reverse = false;
bool joy_enable = true;
void auto_aim()
{
    if(Controller1.ButtonRight.pressing() && re_flag)
    {
        start_aim = !start_aim;
        if(start_aim)
        {
            task a(vis_theta);
            joy_enable = false;
        }
        else
        {
            task::stop(vis_theta);
            joy_enable  = true;
        }
        re_flag = false;
    }
    else if(!Controller1.ButtonRight.pressing()) re_flag = true;
}
void catapult_autoload()
{
    if(Controller1.ButtonR2.pressing() && re_catapult_auto)
    {
        catapult_auto = !catapult_auto;
        re_catapult_auto = false;
    }
    else if(!Controller1.ButtonR2.pressing()) re_catapult_auto = true;
}
void reverse_drive()
{
        if(Controller1.ButtonA.pressing() && re_drive_reverse) //Button toggles reverse driving
        {
            drive_reverse = !drive_reverse;
            re_drive_reverse = false;
        }
        else if (!Controller1.ButtonA.pressing()) re_drive_reverse = true;
}
int drive_l = 0, drive_r = 0;
void drive_tank() //tank_drive for robot and stopping with whatever brakes in the parameter.
{
    if(joy_enable)
    {
    reverse_drive();
    if(drive_reverse) //checks if toggled
    {
        left_axis = -Controller1.Axis2.value();
        right_axis = -Controller1.Axis3.value();
        side_axis = -Controller1.Axis4.value();
    }
    else //if it doesn't toggle
    {
        left_axis  = Controller1.Axis3.value();
        right_axis = Controller1.Axis2.value();
        side_axis = Controller1.Axis4.value();
    }
    if(abs(Controller1.Axis4.value()) > 90 && abs(Controller1.Axis3.value()) < 80)
    {
        drive_r = -side_axis/3;
        drive_l = side_axis/3;
    }
    else
    {
        drive_l = left_axis;
        drive_r = right_axis;
    }
    BackLeft.spin(directionType::fwd,drive_l,velocityUnits::pct);
    FrontLeft.spin(directionType::fwd,drive_l,velocityUnits::pct);
    FrontRight.spin(directionType::fwd,drive_r,velocityUnits::pct);
    BackRight.spin(directionType::fwd,drive_r,velocityUnits::pct);
    }
}

void drive_catapult()
{
    catapult_autoload();
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
        Ballintake.spin(directionType::fwd,-90,velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing())//flip cap if this R2 pressed
    {
        Ballintake.spin(directionType::fwd,90,velocityUnits::pct);
    }
    else // if no buttons pressed, stop the intake
    {
        Ballintake.stop();
    }
}
void drive_stick_() // drive code for stick
{
    if(Controller1.ButtonY.pressing()) //if y pressed, spin stick clockwise
    {
        Stick.spin(directionType::fwd,50,velocityUnits::pct);
    }
    else if (Controller1.ButtonX.pressing()) //if x pressed, spin stick counterclockwise
    {
        Stick.spin(directionType::fwd,-50,velocityUnits::pct);
    }
    else
    {
        Stick.stop(brakeType::coast);
    }
}

void usercontrol( void ) //teleoperator code
{
    stop_chassis();
  // User control code here, inside the loop
   LCD_selector();
   task newtask(check);
   while(true)
   {
       drive_catapult();
       drive_stick_();
       drive_ball_intake();
       drive_tank();
       auto_aim();
   }
}
//Runs the teleop and autonomous in the robot.
int main()
{
    Vision.setWifiMode(vision::wifiMode::off);
    pre_auton();
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.
    while(1) { task::sleep(100); }
}
