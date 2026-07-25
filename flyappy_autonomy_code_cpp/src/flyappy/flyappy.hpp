#pragma once

#include <array>

#include "flyappy/constants.hpp"
#include "flyappy/control/controller.hpp"
#include "flyappy/mapping/obstacle_map.hpp"
#include "flyappy/mapping/occupancy_grid.hpp"
#include "flyappy/planning/path_planner.hpp"

namespace flyappy
{

/// Orchestrates the full pipeline: OccupancyGrid -> ObstacleMap ->
/// PathPlanner -> Controller. ROS-free; fed by setLaserScan() and
/// computeAcceleration() from the ROS wrapper's callbacks.
class Flyappy
{
  public:
    Flyappy();

    /// Reset all internal state (call on game reset / new episode).
    void reset();

    /// Feed a new laser scan (9 ranges, in meters, ordered from -45 to +45
    /// deg). Updates the occupancy grid and rebuilds the obstacle map and
    /// waypoint list.
    void setLaserScan(const std::array<double, kNumRays>& ranges);

    /// Given the current velocity (m/s, noise-free, from /flyappy_vel),
    /// advance the dead-reckoning estimate, run the control cascade
    /// against the current waypoint, and return the acceleration command
    /// to send back to the game.
    AccelerationCommand computeAcceleration(double vx, double vy);

  private:
    // Fixed timestep matching the game's FPS (see flyappy_main_game.py:
    // FPS = 30). Used both for grid dead-reckoning and the controller's
    // position loop; avoids depending on a ROS clock inside this
    // ROS-free class.
    static constexpr double kDt = 1.0 / 30.0;

    // Bird's fixed logical column in the grid: placed a bit inside from
    // the left edge so there is some margin behind the bird too, even
    // though the bird only ever moves forward.
    static constexpr int kBirdColOffset = 20;

    OccupancyGrid grid_;
    ObstacleMap obstacle_map_;
    PathPlanner planner_;
    Controller controller_;

    // Current waypoint list, refreshed every time a new laser scan comes
    // in. The first waypoint is the one actively pursued by the
    // controller.
    std::vector<Waypoint> waypoints_;
};

}  // namespace flyappy
