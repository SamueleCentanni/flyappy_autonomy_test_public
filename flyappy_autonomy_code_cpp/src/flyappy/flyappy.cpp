#include "flyappy/flyappy.hpp"

namespace flyappy
{

Flyappy::Flyappy()
    : grid_(kGridResolution, kGridWidthM, kGridHeightM, kBirdColOffset)
{
}

void Flyappy::reset()
{
    grid_.reset();
    controller_.reset();
    waypoints_.clear();
}

void Flyappy::setLaserScan(const std::array<double, kNumRays>& ranges)
{
    grid_.updateFromLaserScan(ranges);
    obstacle_map_.update(grid_);
    waypoints_ = planner_.computeWaypoints(obstacle_map_.walls(), kGridWidthM, kGridHeightM);
}

AccelerationCommand Flyappy::computeAcceleration(double vx, double vy)
{
    grid_.updatePosition(vx, vy, kDt);

    // Pursue the first waypoint in the list (the nearest one not yet
    // reached). If we don't have any waypoint yet (e.g. very first
    // frames, before any laser scan has arrived), aim straight ahead.
    const double target_y_m = waypoints_.empty() ? 0.0 : waypoints_.front().y_m;

    controller_.updatePositionLoop(target_y_m, vy, kDt);
    return controller_.updateVelocityLoop(vx, vy);
}

}  // namespace flyappy
