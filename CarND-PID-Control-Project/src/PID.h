#ifndef PID_H
#define PID_H

class PID {
public:
  PID();
  virtual ~PID();
  void Init(double Kp, double Ki, double Kd);
  void UpdateError(double cte);
  double TotalError();
  const double maxSteeringAngle = 1.0;
private:
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;
  double Kp;
  double Ki;
  double Kd;
  double previous_error;  
};

#endif