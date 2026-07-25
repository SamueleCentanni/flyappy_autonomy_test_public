#include "flyappy/planning/path_planner.hpp"

#include <cmath>
#include <limits>

#include "flyappy/constants.hpp"

namespace flyappy
{

const Opening& PathPlanner::selectOpening(const Wall& wall, double reference_y_m)
{
    // wall.openings is guaranteed non-empty by the caller.
    size_t best_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < wall.openings.size(); ++i)
    {
        const double dist = std::abs(wall.openings[i].center_y_m - reference_y_m);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    return wall.openings[best_idx];
}

std::vector<Waypoint> PathPlanner::computeWaypoints(const std::vector<Wall>& walls,
                                                      double map_width_m,
                                                      double map_height_m) const
{
    std::vector<Waypoint> waypoints;

    // Reference y for opening selection: starts at 0 (the bird's own
    // altitude, body-frame), then tracks the last chosen opening's center
    // as we move wall by wall, so opening selection stays consistent
    // across consecutive walls instead of jumping erratically.
    double reference_y_m = 0.0;

    for (const auto& wall : walls)
    {
        if (wall.openings.empty())
        {
            // No passable opening found for this wall (e.g. not fully
            // observed yet, or genuinely fully blocked at this height
            // range): skip it, we can't plan through it yet.
            continue;
        }

        const Opening& opening = selectOpening(wall, reference_y_m);

        Waypoint entry;
        entry.x_m = wall.position_x_m - kWaypointXMargin;
        entry.y_m = opening.center_y_m;

        Waypoint exit;
        exit.x_m = wall.position_x_m + wall.thickness_m + kWaypointXMargin;
        exit.y_m = opening.center_y_m;

        // If the bird is already past the entry point (i.e. inside or
        // beyond the wall's x-range), skip the entry waypoint: don't ask
        // the controller to go backward for a waypoint already passed.
        // Body-frame convention: bird is always at x=0.
        if (entry.x_m > 0.0)
        {
            waypoints.push_back(entry);
        }
        waypoints.push_back(exit);

        reference_y_m = opening.center_y_m;
    }

    // Fallback: if no walls are known at all, aim straight ahead at the
    // bird's current altitude, far into the map.
    if (waypoints.empty())
    {
        Waypoint fallback;
        fallback.x_m = map_width_m;
        fallback.y_m = 0.0;
        waypoints.push_back(fallback);
    }

    (void)map_height_m;  // reserved for future use (e.g. clamping y within bounds)

    return waypoints;
}

}  // namespace flyappy
