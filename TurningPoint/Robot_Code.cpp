int drive_val = 90;
int error_auto = 0;
double kp_auto = 0.1;
int auto_val = 0;

int auto_straight()
{
    Gyro.startCalibration();
    while(true)
    {
         error_auto = 0 - Gyro.value(rotationUnits::deg);
         auto_val = kp_auto * error_auto;
         task::sleep(15);    
    }
}

void go_chassis(int degrees)
{
    //task t(auto_straight);
    while(FrontLeft.rotation(rotationUnits::deg) < degrees)
    {
        FrontLeft.spin(directionType::fwd,drive_val-auto_val,velocityUnits::pct);
        FrontRight.spin(directionType::fwd,drive_val+auto_val,velocityUnits::pct);
        BackLeft.spin(directionType::fwd,drive_val-auto_val,velocityUnits::pct);
        BackRight.spin(directionType::fwd,drive_val+auto_val,velocityUnits::pct);
    }
    //task::stop(auto_straight);
}
