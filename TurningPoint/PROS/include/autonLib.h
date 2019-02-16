#ifndef AUTONLIB_H
#define AUTONLIB_H

extern void turn(double degrees, int ms);
extern void arc(double degrees, int ms, bool arc_r);
extern void forward(double encoders, int ms, int ball_intake);
extern void load_catapult();
extern void shoot_catapult();

#endif
