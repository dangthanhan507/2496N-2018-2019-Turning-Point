#ifndef PID_LIB_H
#define PIDLIB_H

class PID
{
  private:
    double kp = 0;
    double ki = 0;
    double kd = 0;
  public:
    double preverror = 0;
    double error = 0;
    double integral = 0;
    double derivative = 0;
    PID(int P, int I, int D)
    {
      kp = P;
      ki = I;
      kd = D;
    }
    int Calc_POS(double target, double input, int input_limit)
    {
      preverror = error;
      error = target - input;
      if (abs(error) < input_limit) integral += error;
      else integral = 0;
      derivative = error - preverror;
      return kp*error + ki*integral + kd*derivative;
    }
};

#endif
