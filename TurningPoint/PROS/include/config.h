#include "main.h"

#ifndef CONFIG_H
#define CONFIG_H

//motors
extern int auton_index;
extern pros::Motor BL;
extern pros::Motor BR;
extern pros::Motor TL;
extern pros::Motor TR;
extern pros::Motor CatL;
extern pros::Motor CatR;
extern pros::Motor Ballintake;
extern pros::Motor Stick;
extern pros::Controller controller;
//sensors
extern pros::ADIDigitalIn Lim_Switch;
extern pros::ADIGyro gyro;
extern pros::Vision vision;

//Vision Signatures

#endif
