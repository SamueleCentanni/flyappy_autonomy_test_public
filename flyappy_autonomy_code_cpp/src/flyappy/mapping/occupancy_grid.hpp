#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "flyappy/constants.hpp"

namespace flyappy
{

/// Binary occupancy grid, body-frame (bird stays at a fixed column),
/// scrolling in x as the bird advances. Backed by a ring buffer on the
/// column axis to make scrolling O(1) instead of O(num_cells).
///
/// Cell values: false = free, true = occupied.
class OccupancyGrid
{
  public:
    /// @param resolution cells per meter
    /// @param width_m map width in meters
    /// @param height_m map height in meters
    /// @param bird_col_offset column index (fixed) occupied by the bird
    OccupancyGrid(double resolution, double width_m, double height_m, int bird_col_offset);

    /// Reset the grid to all-free and the estimated position to the origin.
    void reset();

    /// Advance the internal dead-reckoning position estimate and scroll the
    /// grid if the bird has moved by at least one cell along x.
    /// @param vx, vy velocity in m/s (noise-free, from /flyappy_vel)
    /// @param dt elapsed time in seconds since the last call
    void updatePosition(double vx, double vy, double dt);

    /// Mark cells from a laser scan, ray by ray, using Bresenham tracing
    /// from the bird's current cell to the impact point (or to range_max
    /// if the ray is free).
    /// @param ranges array of kNumRays distances in meters
    void updateFromLaserScan(const std::array<double, kNumRays>& ranges);

    /// Row-major occupancy accessor: true = occupied. Throws/UB if out of
    /// bounds; callers should check numRows()/numCols() first.
    bool occupied(int row, int col) const;

    /// Sum of occupied cells in a column range [col_start, col_end)
    /// across all rows. Used by ObstacleMap for wall detection.
    int columnOccupancySum(int col_start, int col_end) const;

    /// Sum of occupied cells in a row range [row_start, row_end)
    /// restricted to a column range [col_start, col_end). Used by
    /// ObstacleMap for opening detection inside a wall's columns.
    int rowOccupancySum(int row_start, int row_end, int col_start, int col_end) const;

    int numRows() const { return num_rows_; }
    int numCols() const { return num_cols_; }
    double resolution() const { return resolution_; }

    /// Bird's fixed column index in the grid.
    int birdCol() const { return bird_col_; }
    /// Bird's current row index in the grid (tracks vertical dead-reckoning).
    int birdRow() const { return bird_row_; }

    /// Convert a grid row/col to a body-frame position in meters, relative
    /// to the bird's current position (x=0 at bird_col, y=0 at bird_row).
    double colToRelX(int col) const;
    double rowToRelY(int row) const;

  private:
    /// Map a logical column index (0 = leftmost visible column) to its
    /// physical index in the ring buffer.
    int physicalCol(int logical_col) const;

    /// Bresenham line traversal from (row0,col0) to (row1,col1), calling
    /// visit(row, col, is_endpoint) for every intermediate cell.
    template <typename Visitor>
    void traceLine(int row0, int col0, int row1, int col1, Visitor&& visit) const;

    double resolution_;
    int num_rows_;
    int num_cols_;
    int bird_col_;  // fixed logical column of the bird

    // Ring buffer: physical storage, row-major. origin_col_ is the physical
    // index corresponding to logical column 0.
    std::vector<std::vector<bool>> cells_;  // cells_[row][physical_col]
    int origin_col_ = 0;

    // Dead-reckoning state
    double bird_x_m_ = 0.0;  // absolute x position estimate (meters)
    double bird_y_m_ = 0.0;  // absolute y position estimate (meters), grid-relative
    int bird_row_ = 0;
    double leftover_x_m_ = 0.0;  // sub-cell x accumulator for smooth scrolling
};

}  // namespace flyappy
