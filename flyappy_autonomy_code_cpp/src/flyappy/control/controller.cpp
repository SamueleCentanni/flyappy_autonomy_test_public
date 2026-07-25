#include "flyappy/control/controller.hpp"

#include <algorithm>
#include <cmath>

#include "flyappy/constants.hpp"

namespace flyappy
{

void Controller::reset()
{
    integral_y_ = 0.0;
    ref_vy_ = 0.0;
}

void Controller::updatePositionLoop(double target_y_m, double vy, double dt)
{
    // Body-frame convention: the bird is always at y=0 by definition, so
    // the position error is simply the target itself.
    const double error_y = target_y_m;

    if (std::abs(error_y) < kPositionErrorDeadband)
    {
        // Close enough: stop commanding vertical motion to avoid jitter
        // around the target, and don't let the integral term keep
        // accumulating while sitting inside the deadband.
        ref_vy_ = 0.0;
        integral_y_ = 0.0;
        return;
    }

    integral_y_ += error_y * dt;
    integral_y_ = std::clamp(integral_y_, -kIntegralPositionSaturation, kIntegralPositionSaturation);

    // Derivative on measurement: the error shrinks as the bird approaches
    // the target primarily because of its own velocity, so -vy is a
    // direct, non-noisy stand-in for d(error)/dt that isn't corrupted by
    // abrupt setpoint changes (avoids the classic "derivative kick" that
    // differentiating the error directly would cause every time the
    // target waypoint changes).
    const double derivative_term = -vy;

    ref_vy_ = (kKpPosition * error_y) + (kKiPosition * integral_y_) + (kKdPosition * derivative_term);
}

AccelerationCommand Controller::updateVelocityLoop(double vx, double vy) const
{
    AccelerationCommand cmd;

    cmd.ay = kKpVelocity * (ref_vy_ - vy);
    cmd.ay = std::clamp(cmd.ay, -kAccYLimit, kAccYLimit);

    cmd.ax = kKpVelocity * (kTargetVx - vx);
    cmd.ax = std::clamp(cmd.ax, -kAccXLimit, kAccXLimit);

    return cmd;
}

}  // namespace flyappy
