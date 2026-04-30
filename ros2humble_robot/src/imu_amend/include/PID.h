#pragma once

class PID {
public:
  PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      prev_error_(0.0), integral_(0.0) {}

  double calculate(double setpoint, double pv) {
    double error = setpoint - pv;
    
    // 积分项
    integral_ += error;
    
    // 微分项
    double derivative = error - prev_error_;
    prev_error_ = error;
    
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};