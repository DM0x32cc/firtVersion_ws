#ifndef OFFBOARD_PID_CONTROLLER_HPP
#define OFFBOARD_PID_CONTROLLER_HPP

#include<algorithm>
namespace offboard
{

class PIDController {
public:
    PIDController(double kp, double ki, double kd,double i_max = 0.6 )
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), i_max_(i_max) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -i_max_, i_max_);
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
    double i_max_;
};

} // namespace offboard

#endif // OFFBOARD_PID_CONTROLLER_HPP 