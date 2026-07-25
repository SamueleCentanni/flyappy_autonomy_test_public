#pragma once

#include <vector>

#include "flyappy/mapping/occupancy_grid.hpp"

namespace flyappy
{

/// A vertical opening within a wall: spans rows [row_start, row_end)
/// (grid-relative), with a cached center in meters (body-frame, relative
/// to the bird).
struct Opening
{
    int row_start = 0;
    int row_end = 0;      // exclusive
    double center_y_m = 0.0;  // body-frame, relative to bird's current row
};

/// A wall: a contiguous run of columns whose occupancy sum exceeds
/// kWallThresholdCells, up to kMaxWallThicknessM thick. Holds its detected
/// openings.
struct Wall
{
    int col_start = 0;
    int col_end = 0;  // exclusive
    double position_x_m = 0.0;   // body-frame x of col_start, relative to bird
    double thickness_m = 0.0;
    std::vector<Opening> openings;
};

/// Analyzes an OccupancyGrid to recognize walls and their openings as
/// abstract objects, instead of reasoning cell-by-cell.
class ObstacleMap
{
  public:
    /// Re-scan the grid and rebuild the list of walls/openings from
    /// scratch. Call once per control cycle after the grid has been
    /// updated with the latest laser scan.
    void update(const OccupancyGrid& grid);

    const std::vector<Wall>& walls() const { return walls_; }

  private:
    /// Scan column occupancy sums to find contiguous wall segments.
    std::vector<Wall> detectWalls(const OccupancyGrid& grid) const;

    /// Within a wall's column range, scan row occupancy sums to find
    /// contiguous zero-sum (free) row segments at least kMinOpeningSizeM
    /// tall.
    std::vector<Opening> detectOpenings(const OccupancyGrid& grid, const Wall& wall) const;

    std::vector<Wall> walls_;
};

}  // namespace flyappy
