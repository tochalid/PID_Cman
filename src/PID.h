#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /*
   * Errors
   */

  // Vector of proportional/integral/differential error terms
  std::vector<double> errors;

  /*
   * Coefficients
   */
  // tau_pid
  double Kp_;
  double Ki_;
  double Kd_;

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
  std::vector<double> UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();
};

#endif /* PID_H */
