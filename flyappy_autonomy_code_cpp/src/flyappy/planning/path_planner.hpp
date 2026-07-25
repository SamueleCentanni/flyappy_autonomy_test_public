#pragma once

#include <vector>

#include "flyappy/mapping/obstacle_map.hpp"

namespace flyappy
{

/// A single waypoint in body-frame coordinates (meters), relative to the
/// bird's current position.
struct Waypoint
{
    double x_m = 0.0;
    double y_m = 0.0;
};

/// Computes a waypoint list from the current wall/opening layout: an
/// entry and exit point per wall, centered on the chosen opening, plus a
/// fallback far-ahead point when no walls are known.
class PathPlanner
{
  public:
    /// @param bird_x_m current bird x (body-frame, always 0 by convention
    ///        since walls are already given relative to the bird)
    /// @param map_width_m, map_height_m grid dimensions in meters, used for
    ///        the fallback waypoint
    std::vector<Waypoint> computeWaypoints(const std::vector<Wall>& walls, double map_width_m,
                                            double map_height_m) const;

  private:
    /// If a wall has multiple openings, pick the one closest (in y) to the
    /// reference y. When there is no previous waypoint yet (first wall
    /// ever encountered), callers should pass 0.0 (grid-center in
    /// body-frame y, since the bird itself defines y=0) as the reference.
    static const Opening& selectOpening(const Wall& wall, double reference_y_m);
};

}  // namespace flyappy
