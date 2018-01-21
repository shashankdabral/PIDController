#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  /* Initialize prev_cte to 0 for D controller */
  Kp_ = Kp; 
  Ki_ = Ki;
  Kd_ = Kd;
  prev_cte_ = 0;
  int_cte_  = 0;
}

void PID::UpdateError(double cte) {
    double diff_cte;
    p_error = -Kp_ * cte;

    diff_cte = cte - prev_cte_;
    d_error  = -Kd_ * diff_cte;  

    int_cte_  += cte;
    i_error   = -Ki_ * int_cte_;

    prev_cte_ = cte;
}


double PID::TotalError() {
    double total_error;
    total_error = p_error + i_error + d_error;
    return total_error;
}

