#include "main.h"
#include "customLib/config.h"

//motors
int auton_index = 0;
pros::Motor BL(12, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BR(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TL(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TR(8, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor CatL(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor CatR(1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Ballintake(18, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Stick(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//sensors
pros::ADIDigitalIn Lim_Switch(1);
pros::ADIGyro gyro(8);
pros::Vision vision(2);

//Vision Signatures

pros::vision_signature_s_t REDBOI = pros::Vision::signature_from_utility(1,5393,10063,7728,-265,265,0,0.8,1);
pros::vision_signature_s_t GREENBOI = pros::Vision::signature_from_utility(3,-2193,-1461,-1828,-5121,-3935,-4528,3,1);
pros::vision_color_code_t REDFLAG = vision.create_color_code(1,3);
