#include "PID.h"

#include <iostream>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  
  last_timestamp_ = std::chrono::system_clock::now();
  last_cte_ = 0.0;
  int_cte_ = 0.0;
  
  twiddle_state_ = twiddle_state::start;
  twiddle_index_ = 0;
  twiddle_p_[0] = Kp_;
  twiddle_p_[1] = Kd_;
  twiddle_p_[2] = Ki_;
  twiddle_dp_[0] = 0.1;
  twiddle_dp_[1] = 0.1;
  twiddle_dp_[2] = 0.1;
  best_error_ = std::numeric_limits<double>::max();
}

void PID::OnConnection() {
  // Reset timing activity.
  last_timestamp_ = std::chrono::system_clock::now();
  // Reset twiddle state.
  twiddle_state_ = twiddle_state::start;
  twiddle_index_ = 0;
  best_error_ = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  // Calculate the elapsed time in seconds since the last update.
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timestamp_).count() / 1000.0;
  //std::cout << dt << std::endl;
  
  // Apply twiddle algorithm to tune PID coefficients.
  Twiddle(cte);
  
  // Apply the PID coefficients to calculate the new error terms.
  p_error_ = -Kp_ * cte;
  d_error_ = -Kd_ * (cte - last_cte_) / dt;
  i_error_ = -Ki_ * int_cte_;

  // Save state for next update.
  last_timestamp_ = now;
  last_cte_ = cte;
  int_cte_ += cte * dt;
}

double PID::TotalError() {
  return p_error_ + d_error_ + i_error_;
}

void PID::Twiddle(double cte) {
  double err = cte * cte;
  
  switch (twiddle_state_) {
  case twiddle_state::start:
    twiddle_p_[twiddle_index_] += twiddle_dp_[twiddle_index_];
    twiddle_state_ = twiddle_state::eval_high;
    break;
  case twiddle_state::eval_high:
    if (err < best_error_) {
      best_error_ = err;
      twiddle_dp_[twiddle_index_] *= 1.1;
      twiddle_state_ = twiddle_state::start;
      twiddle_index_ = (twiddle_index_ + 1) % 3;
    } else {
      twiddle_p_[twiddle_index_] -= 2 * twiddle_dp_[twiddle_index_];
      twiddle_state_ = twiddle_state::eval_low;
    }
    break;
  case twiddle_state::eval_low:
    if (err < best_error_) {
      best_error_ = err;
      twiddle_dp_[twiddle_index_] *= 1.1;
      twiddle_state_ = twiddle_state::start;
      twiddle_index_ = (twiddle_index_ + 1) % 3;      
    } else {
      twiddle_p_[twiddle_index_] += twiddle_dp_[twiddle_index_];
      twiddle_dp_[twiddle_index_] *= 0.9;
      twiddle_state_ = twiddle_state::start;
      twiddle_index_ = (twiddle_index_ + 1) % 3;      
    }
    break;
  }
  
  // Copy back the current coefficients.
  Kp_ = twiddle_p_[0];
  Kd_ = twiddle_p_[1];
  Ki_ = twiddle_p_[2];
  
  std::cout << Kp_ << ", " << Ki_ << ", " << Kd_ << " === " << best_error_ << std::endl;
}

