using namespace vex;
brain Brain;
motor BackRight (PORT11, gearSetting::ratio18_1,true);
motor Stick (PORT7, gearSetting::ratio18_1,false);
motor FrontLeft (PORT9, gearSetting::ratio18_1,false);
motor FrontRight (PORT8, gearSetting::ratio18_1,true);
motor BackLeft (PORT12, gearSetting::ratio18_1,false);
motor CatapultLeft (PORT10, gearSetting::ratio18_1,false);
motor Ballintake (PORT18, gearSetting::ratio18_1,false);
motor CatapultRight (PORT1, gearSetting::ratio18_1,true);

controller Controller1;
competition Competition;
limit Limit1 = limit(Brain.ThreeWirePort.A);
gyro Gyro = gyro(Brain.ThreeWirePort.H);
accelerometer Accelerometer = accelerometer(Brain.ThreeWirePort.G);

// sub class motor to add new method
//adding to the namespace
namespace vex 
{
  class motor_special : public motor 
  {
    private:
      uint32_t  _local_index; //port number
    public:
      //defining constructors of motor voltage class
      motor_special( int32_t index ) :  motor( index ), _local_index(index) {};  
      ~motor_special() {};
      // Allow overloading of base class methods
      using motor::spin;
      // voltage can be +/- 12.0 volts or +/-12000 mV
      void spin( directionType dir, double voltage, voltageUnits units ) 
      {
        // convert volts to mV if necessary
        int32_t voltage_mv = (units == voltageUnits::volt ? voltage * 1000.0 : voltage );

        // flip based on direction flag
        voltage_mv = (dir == directionType::fwd ? voltage_mv : -(voltage_mv) );

        if( voltage_mv == 0 )  stop(); 
        else vexMotorVoltageSet( _local_index, voltage_mv ); // send mV value to control motor open loop
      }
  };
}
//defining the chassis motors in voltage class to use voltage 
motor_special BR( PORT11 );
motor_special BL( PORT12 );
motor_special FR( PORT8 );
motor_special FL( PORT9 );

//vision sensor stuff
//The Vision Sensor is sensitive to different levels of light.
vision::signature F_BLUE (1, -2903, 671, -1116, 173, 13493, 6833, 0.8, 0);
vision::signature F_GREEN (2, -2301, -1147, -1724, -4463, -1875, -3169, 3, 0);
vision::signature F_RED (3, 9039, 12289, 10664, -373, 461, 44, 3, 0);
vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);   
vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision (vex::PORT20, 50, F_BLUE, F_GREEN, F_RED, SIG_4, SIG_5, SIG_6, SIG_7);
