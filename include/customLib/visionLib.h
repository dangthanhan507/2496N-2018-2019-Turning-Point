#include "main.h"
#include "config.h"

#ifndef VISIONLIB_H
#define VISIONLIB_H

extern pros::vision_object_s_t vision_capture(pros::vision_color_code_t COLOR_CODE);
extern void X_AIM(pros::vision_color_code_t COLOR_CODE);


#endif
