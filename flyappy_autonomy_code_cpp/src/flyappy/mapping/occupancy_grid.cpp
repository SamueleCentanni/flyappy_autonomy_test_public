#include "flyappy/mapping/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace flyappy
{

OccupancyGrid::OccupancyGrid(double resolution, double width_m, double height_m,
                              int bird_col_offset)
    : resolution_(resolution),
      num_rows_(static_cast<int>(std::round(height_m * resolution))),
      num_cols_(static_cast<int>(std::round(width_m * resolution))),
      bird_col_(bird_col_offset)
{
    cells_.assign(static_cast<size_t>(num_rows_), std::vector<bool>(static_cast<size_t>(num_cols_), false));
    bird_row_ = num_rows_ / 2;
}

void OccupancyGrid::reset()
{
    for (auto& row : cells_)
    {
        std::fill(row.begin(), row.end(), false);
    }
    origin_col_ = 0;
    bird_x_m_ = 0.0;
    bird_y_m_ = 0.0;
    bird_row_ = num_rows_ / 2;
    leftover_x_m_ = 0.0;
}

int OccupancyGrid::physicalCol(int logical_col) const
{
    int phys = (origin_col_ + logical_col) % num_cols_;
    if (phys < 0)
    {
        phys += num_cols_;
    }
    return phys;
}

bool OccupancyGrid::occupied(int row, int col) const
{
    if (row < 0 || row >= num_rows_ || col < 0 || col >= num_cols_)
    {
        return true;  // treat out-of-bounds as occupied (safe default)
    }
    return cells_[static_cast<size_t>(row)][static_cast<size_t>(physicalCol(col))];
}

void OccupancyGrid::updatePosition(double vx, double vy, double dt)
{
    // Integrate velocity (dead-reckoning, exact since /flyappy_vel is
    // noise-free).
    bird_x_m_ += vx * dt;
    bird_y_m_ += vy * dt;
    leftover_x_m_ += vx * dt;

    // Vertical motion: shift the bird's row estimate. We keep bird_row_ as
    // a continuous quantity via rounding here; sub-cell vertical position
    // is not separately tracked since openings only need row-resolution.
    const double bird_row_exact = static_cast<double>(num_rows_) / 2.0 - bird_y_m_ * resolution_;
    bird_row_ = static_cast<int>(std::lround(bird_row_exact));
    bird_row_ = std::clamp(bird_row_, 0, num_rows_ - 1);

    // Horizontal motion: scroll the ring buffer by whole cells, keep the
    // sub-cell remainder for the next call (avoids drift from repeated
    // rounding).
    const double cell_size = 1.0 / resolution_;
    int cells_to_scroll = static_cast<int>(leftover_x_m_ / cell_size);
    if (cells_to_scroll > 0)
    {
        leftover_x_m_ -= cells_to_scroll * cell_size;

        if (cells_to_scroll >= num_cols_)
        {
            // Moved more than the whole grid width in one step: everything
            // is stale, clear it all and start fresh.
            reset();
            return;
        }

        // Advance the ring buffer origin first, so that logical column
        // indices below refer to the grid *after* scrolling.
        origin_col_ = physicalCol(cells_to_scroll);

        // Clear the columns that are now newly revealed on the right edge
        // (they represent unobserved space and must not keep stale
        // occupancy from whatever used to be there in the ring buffer).
        for (int i = 0; i < cells_to_scroll; ++i)
        {
            const int logical_col_to_clear = num_cols_ - cells_to_scroll + i;
            const int phys = physicalCol(logical_col_to_clear);
            for (auto& row : cells_)
            {
                row[static_cast<size_t>(phys)] = false;
            }
        }
    }
}

double OccupancyGrid::colToRelX(int col) const
{
    return static_cast<double>(col - bird_col_) / resolution_;
}

double OccupancyGrid::rowToRelY(int row) const
{
    // Row increases downward in the grid, y increases upward physically.
    return static_cast<double>(bird_row_ - row) / resolution_;
}

template <typename Visitor>
void OccupancyGrid::traceLine(int row0, int col0, int row1, int col1, Visitor&& visit) const
{
    int dx = std::abs(col1 - col0);
    int dy = -std::abs(row1 - row0);
    int sx = col0 < col1 ? 1 : -1;
    int sy = row0 < row1 ? 1 : -1;
    int err = dx + dy;

    int row = row0;
    int col = col0;

    while (true)
    {
        const bool is_endpoint = (row == row1 && col == col1);
        visit(row, col, is_endpoint);
        if (is_endpoint)
        {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            col += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            row += sy;
        }
    }
}

void OccupancyGrid::updateFromLaserScan(const std::array<double, kNumRays>& ranges)
{
    for (int i = 0; i < kNumRays; ++i)
    {
        const double range = ranges[static_cast<size_t>(i)];
        const bool hit = range < (kRangeMax - kRangeMaxTol);
        const double effective_range = hit ? range : kRangeMax;

        const double angle_deg = -kFovDeg / 2.0 + i * kAngleStepDeg;
        const double angle_rad = angle_deg * M_PI / 180.0;

        const double dx_m = effective_range * std::cos(angle_rad);
        const double dy_m = effective_range * std::sin(angle_rad);

        const int end_col = bird_col_ + static_cast<int>(std::round(dx_m * resolution_));
        // y positive = up = row decreases
        const int end_row = bird_row_ - static_cast<int>(std::round(dy_m * resolution_));

        const int clamped_end_col = std::clamp(end_col, 0, num_cols_ - 1);
        const int clamped_end_row = std::clamp(end_row, 0, num_rows_ - 1);

        traceLine(bird_row_, bird_col_, clamped_end_row, clamped_end_col,
                  [&](int row, int col, bool is_endpoint)
                  {
                      if (row < 0 || row >= num_rows_ || col < 0 || col >= num_cols_)
                      {
                          return;
                      }
                      const int phys = physicalCol(col);
                      const bool already_occupied = cells_[static_cast<size_t>(row)][static_cast<size_t>(phys)];
                      if (is_endpoint && hit)
                      {
                          cells_[static_cast<size_t>(row)][static_cast<size_t>(phys)] = true;
                      }
                      else if (!already_occupied)
                      {
                          // Cell traversed as free by this ray: leave it
                          // free, but only if no other ray in this same
                          // scan already marked it occupied. Occupied
                          // always wins over free within one scan,
                          // regardless of ray iteration order.
                          cells_[static_cast<size_t>(row)][static_cast<size_t>(phys)] = false;
                      }
                  });
    }
}

int OccupancyGrid::columnOccupancySum(int col_start, int col_end) const
{
    int sum = 0;
    for (int col = col_start; col < col_end; ++col)
    {
        if (col < 0 || col >= num_cols_)
        {
            continue;
        }
        const int phys = physicalCol(col);
        for (const auto& row : cells_)
        {
            if (row[static_cast<size_t>(phys)])
            {
                ++sum;
            }
        }
    }
    return sum;
}

int OccupancyGrid::rowOccupancySum(int row_start, int row_end, int col_start, int col_end) const
{
    int sum = 0;
    for (int row = row_start; row < row_end; ++row)
    {
        if (row < 0 || row >= num_rows_)
        {
            continue;
        }
        for (int col = col_start; col < col_end; ++col)
        {
            if (col < 0 || col >= num_cols_)
            {
                continue;
            }
            const int phys = physicalCol(col);
            if (cells_[static_cast<size_t>(row)][static_cast<size_t>(phys)])
            {
                ++sum;
            }
        }
    }
    return sum;
}

}  // namespace flyappy
