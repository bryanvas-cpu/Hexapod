// pid_controller.hpp
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PID {
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        // PID formula: u(t) = Kp * e(t) + Ki * ∫e(t) dt + Kd * d(e)/dt
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;     // PID coefficients
    double prev_error_;        // Previous error for derivative calculation
    double integral_;          // Integral of the error
};

#endif // PID_CONTROLLER_HPP
