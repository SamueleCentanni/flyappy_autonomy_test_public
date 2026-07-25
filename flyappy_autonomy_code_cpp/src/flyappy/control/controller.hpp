#pragma once

#include "flyappy/planning/path_planner.hpp"

namespace flyappy
{

struct AccelerationCommand
{
    double ax = 0.0;
    double ay = 0.0;
};

/// Cascade controller: position PID (y only) -> velocity P (x and y) ->
/// acceleration command. X cruise speed is a constant target; only Y is
/// actively steered toward the current waypoint.
class Controller
{
  public:
    /// Reset integral/derivative state (call on game reset).
    void reset();

    /// Position control loop (Y only). Updates the internal velocity
    /// reference for Y based on the error to the target waypoint. Uses
    /// "derivative on measurement" (derivative term based on -vy, the
    /// measured velocity) instead of deriving the position error, to
    /// avoid a derivative kick when the target waypoint changes abruptly.
    /// @param target_y_m waypoint y, body-frame relative to the bird
    /// @param vy measured vertical velocity (m/s, noise-free)
    /// @param dt elapsed time in seconds since the last call
    void updatePositionLoop(double target_y_m, double vy, double dt);

    /// Velocity control loop (X and Y). Computes the acceleration command
    /// from the current velocity reference and the measured velocity.
    /// @param vx, vy measured velocity (m/s, noise-free, from /flyappy_vel)
    AccelerationCommand updateVelocityLoop(double vx, double vy) const;

  private:
    // Position loop state (PID on y). No prev_error_y_ needed anymore:
    // the derivative term now uses -vy directly (derivative on
    // measurement) instead of numerically differentiating the error.
    double integral_y_ = 0.0;

    // Velocity reference produced by the position loop, consumed by the
    // velocity loop.
    double ref_vy_ = 0.0;
};

}  // namespace flyappy
