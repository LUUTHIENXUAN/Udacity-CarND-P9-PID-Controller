#include <iostream>
#include <math.h>
#include <vector>

#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle.
  */
  int max_time_step;
  int current_stage;
  int index;
  double err, best_error;
  bool twiddle_intialized, twiddle_completed;
  std::vector<double> dparams, params;

  double run(double cte, int current_time_step);
  void twiddle(double tol, double err);


};

#endif /* PID_H */
