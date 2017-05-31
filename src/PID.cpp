#include "PID.h"

#include <iostream>

static const int MAX_TWIDDLE_STEPS = 1000;

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
  
  twiddle_enabled_ = false;
}

void PID::InitTwiddle(double Kpd, double Kid, double Kdd) {
  twiddle_enabled_ = true;
  twiddle_state_ = twiddle_state::start_high;
  twiddle_index_ = 0;
  twiddle_p_[0] = Kp_;
  twiddle_p_[1] = Kd_;
  twiddle_p_[2] = Ki_;
  twiddle_dp_[0] = Kpd;
  twiddle_dp_[1] = Kid;
  twiddle_dp_[2] = Kdd;
  twiddle_best_err_ = std::numeric_limits<double>::max();
  twiddle_curr_err_ = 0;
  twiddle_step_ = 0;
}

void PID::OnConnection() {
  // Reset timing activity.
  last_timestamp_ = std::chrono::system_clock::now();
  if (twiddle_enabled_) {
    twiddle_step_ = 0;
    twiddle_curr_err_ = 0;
    TwiddleStart();    
  }
}

void PID::UpdateError(double cte) {
  // Calculate the elapsed time in seconds since the last update.
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timestamp_).count() / 1000.0;
  
  // Accumulate the twiddle error so far.
  if (twiddle_enabled_) {
    twiddle_step_++;
    if (twiddle_step_ < MAX_TWIDDLE_STEPS) {
      twiddle_curr_err_ += cte * cte;    
    } else if (twiddle_step_ == MAX_TWIDDLE_STEPS) {
      TwiddleFinish();
    }    
  }
  
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

void PID::TwiddleStart() {
  switch (twiddle_state_) {
  case twiddle_state::start_high:
    twiddle_p_[twiddle_index_] += twiddle_dp_[twiddle_index_];
    twiddle_state_ = twiddle_state::eval_high;
    break;
  case twiddle_state::start_low:
    twiddle_p_[twiddle_index_] -= 2 * twiddle_dp_[twiddle_index_];
    twiddle_state_ = twiddle_state::eval_low;
    break;
  case twiddle_state::eval_high:
  case twiddle_state::eval_low:
    std::cout << "Unexpect state at start" << std::endl;
    break;
  }
  
  // Copy back the current coefficients.
  Kp_ = twiddle_p_[0];
  Kd_ = twiddle_p_[1];
  Ki_ = twiddle_p_[2];
}

void PID::TwiddleFinish() {
  switch (twiddle_state_) {
  case twiddle_state::start_high:
  case twiddle_state::start_low:
      std::cout << "Unexpect state at finish" << std::endl;
      break;
  case twiddle_state::eval_high:
    if (twiddle_curr_err_ < twiddle_best_err_) {
      twiddle_best_err_ = twiddle_curr_err_;
      twiddle_dp_[twiddle_index_] *= 1.1;
      twiddle_state_ = twiddle_state::start_high;
      twiddle_index_ = (twiddle_index_ + 1) % 3;
    } else {
      twiddle_state_ = twiddle_state::start_low;
    }
    break;
  case twiddle_state::eval_low:
    if (twiddle_curr_err_ < twiddle_best_err_) {
      twiddle_best_err_ = twiddle_curr_err_;
      twiddle_dp_[twiddle_index_] *= 1.1;
      twiddle_state_ = twiddle_state::start_high;
      twiddle_index_ = (twiddle_index_ + 1) % 3;      
    } else {
      twiddle_p_[twiddle_index_] += twiddle_dp_[twiddle_index_];
      twiddle_dp_[twiddle_index_] *= 0.9;
      twiddle_state_ = twiddle_state::start_high;
      twiddle_index_ = (twiddle_index_ + 1) % 3;      
    }
    break;
  }
  std::cout << "P: " << Kp_ << ", "
            << "I: " << Ki_ << ", "
            << "D: " << Kd_ << ", "
            << "Error: " << twiddle_curr_err_ << std::endl;
}
