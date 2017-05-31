#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
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
  
  /**
   * Initialize Twiddle and enable experimentation.
   */
  void InitTwiddle(double Kpd, double Kid, double Kdd);
  
  /**
   * Reset state when connection is established.
   */
  void OnConnection();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
private:
  std::chrono::system_clock::time_point last_timestamp_;
  double last_cte_;
  double int_cte_;
  
  enum class twiddle_state {
    start_high,
    start_low,
    eval_high,
    eval_low
  };
  
  twiddle_state twiddle_state_;
  int twiddle_index_;
  double twiddle_p_[3];
  double twiddle_dp_[3];
  double twiddle_best_err_;
  double twiddle_curr_err_;
  int twiddle_step_;
  bool twiddle_enabled_;
  
  void TwiddleStart();
  void TwiddleFinish();
};

#endif /* PID_H */
