#ifndef OFFBOARD_PID_CONTROLLER_HPP
#define OFFBOARD_PID_CONTROLLER_HPP

namespace offboard
{

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
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
};

} // namespace offboard

#endif // OFFBOARD_PID_CONTROLLER_HPP 