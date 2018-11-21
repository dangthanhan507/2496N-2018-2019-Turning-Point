#include "robot-config.h"

vex::competition    Competition;
/*
///////////////////////////////////
-----------------------------------
Motor Configuration:
4 motor chassis
2 motor catapult
1 motor stick
1 motor ball intake
Brain Ports:
1. BackLeft
2. BackRight
3. FrontLeft
4. FrontRight
5. Left Catapult
6. Right Catapult
7. Stick
8. Ball Intake
-----------------------------------
///////////////////////////////////
*/

/*
"""///////////////////////////"""
         Control Loops
"""///////////////////////////"""
*/



/*
"""///////////////////////////"""
           Auton Code
"""///////////////////////////"""
*/

void pre_auton( void ) //first time setup for robot
{  
}  

void autonomous( void ) //autonomous code that runs for 15 seconds
{
  
}

/*
"""///////////////////////////"""
           Drive Code
"""///////////////////////////"""
*/

bool drive_reverse = false;
int left_axis = 0;
int right_axis = 0;
void drive_tank(brakeType type) //tank_drive for robot and stopping with whatever brakes in the parameter.
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
    BackRight.spin(directionType::fwd,right_axis,velocityUnits::pct);
}
void drive_catapult() //catapult code for driving
{
    if(Controller1.ButtonA.pressing())//hold onto A to move catapult
    {
        CatapultLeft.spin(directionType::fwd,100,velocityUnits::pct);
        CatapultRight.spin(directionType::fwd,100,velocityUnits::pct);   
    }
    else//if button is not pressed, stop the motors.
    {
        CatapultLeft.stop();
        CatapultRight.stop();
    }
}
void drive_ball_intake() // drive code for ball intake
{
    if(Controller1.ButtonR1.pressing())//intake the ball if this R1 presseed
    {
        Ballintake.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing())//flip cap if this R2 pressed
    {
        Ballintake.spin(directionType::fwd,-100,velocityUnits::pct);
    }
    else // if no buttons pressed, stop the intake 
    {
        Ballintake.stop();
    }
}
void drive_stick_() // drive code for stick
{
    if(Controller1.ButtonL1.pressing()) //if L1 pressed, spin stick clockwise
    {
        Stick.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else if (Controller1.ButtonL2.pressing()) //if l2 pressed, spin stick counterclockwise
    {
        Stick.spin(directionType::fwd,-100,velocityUnits::pct);
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
       drive_tank(coast);
       drive_catapult();
   }
}

//Runs the teleop and autonomous in the robot.
int main() 
{
    pre_auton();
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) { vex::task::sleep(100); }
}
