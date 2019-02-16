#include "main.h"
#include "config.h"

#ifndef DRIVE_H
#define DRIVE_H

bool catapult_auto = true;
bool catapult_load = true;

//********************* DRIVE CATAPULT *****************************
void drive_catapult()
{
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) catapult_auto = !catapult_auto;
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
  {
    CatL.move(127);
    CatR.move(127);
    catapult_load = true;
  }
  else
  {
    if(catapult_auto && catapult_load)
    {
      CatL.move(127);
      CatR.move(127);
      if(Lim_Switch.get_value() == 1) catapult_load = false;
    }
    else
    {
      CatL.move(0);
      CatR.move(0);
    }
  }
}

//*********************** DRIVE BALL INTAKE ************************
void drive_ball_intake() {
// Change motor ports to be with the current robot motor ports 
// change the rpm for the correct gear set 
  if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_L1) ) {
    intake_motor.move_velocity(100);// configure the RPM 
    }
  else if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_L2) ) {
    intake_motor.move_velocity(-100);// configure the rpm
    }
  else  {
    intake_motor.move_velocty(0); // stops the motor 
    }
}

//***************************** DRIVE STICK *************************
void drive_stick()// drive for stick {
  if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_Y)) {
    Stick.move_velocity(50); 
    }
  else if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_X)) {
    Stick.move_velocity(-50);
    }
  else {
    Stick.move_velocity(0);
    }
}

///**************************** DRIVE TANK **************************
void drive_ball_intake () {
//decleration
// Change motor ports to be with the current robot ports
// chagne the rpm for the correct gear set
if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_L1) ){
	intake_motor.move_velocity(100);// configure the RPM
	}
else if (master.get_digital_new_press (E_CONTROLLER_DIGITAL_L2) ){
	intake_motor.move_velocity(-100);// configure the rpm
	}
else{
	intake_motor.move_velocty(0); // stops the motor
	}
}
//*********************** DRIVE TANKS FUNCTION ********************************
void drive_tank(){
	reverse_drive();
	if(drive_reverse) { // checks if toggled
		 // DECLARE variable in main()
		 left_axis = -E_CONTROLLER_ANALOG_LEFT_Y;
		 right_axis = -E_CONTROLLER_ANALOG_RIGHT_Y;
		 side_axis = -E_CONTROLLER_ANALOG_LEFT_X;
		}
	else {//if it is toggled
		 left_axis = E_CONTROLLER_ANALOG_LEFT_Y;
		 right_axis = E_CONTROLLER_ANALOG_RIGHT_Y;
		 side_axis = E_CONTROLLER_ANALOG_LEFT_X;
  }
	if(abs(E_CONTROLLER_ANALOG_LEFT_X) > 90 && abs(E_CONTROLLER_ANALOG_LEFT_X) > 80) {
		//more unknown variables needed to be declared
		drive_r = -side_axis/3;
		drive_l = side_axis/3;
	}
	else {
		drive_l = left_axis;
		drive_r = right_axis;
	}
	//MOTORS that need to be declared
	BackLeft.move(drive_l);
	FrontLeft.move(drive_l);
	FrontRight.move(drive_r);
	BackRight.move(drive_r);
}
#endif
