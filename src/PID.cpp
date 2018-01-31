#include "PID.h"

using namespace std;

/*
 * REVIEW: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // prepare the error container
  errors.resize(3);
  // init the params
  Kp_ = Kp;  // taup
  Ki_ = Ki;  // taui
  Kd_ = Kd;  // taud
}

std::vector<double> PID::UpdateError(double cte) {
  // Calculate differential error term with previous proportinal error
  errors[2] = cte - errors[0];  // d

  errors[0] = cte;   // p
  errors[1] += cte;  // i
  return errors;
}

double PID::TotalError() {
  double total_error = -(Kp_ * errors[0] + Ki_ * errors[1] + Kd_ * errors[2]);
  return total_error;
}