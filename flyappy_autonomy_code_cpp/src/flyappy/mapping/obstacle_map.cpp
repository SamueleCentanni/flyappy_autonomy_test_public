#include "flyappy/mapping/obstacle_map.hpp"

#include "flyappy/constants.hpp"

namespace flyappy
{

void ObstacleMap::update(const OccupancyGrid& grid)
{
    walls_ = detectWalls(grid);
    for (auto& wall : walls_)
    {
        wall.openings = detectOpenings(grid, wall);
    }
}

std::vector<Wall> ObstacleMap::detectWalls(const OccupancyGrid& grid) const
{
    std::vector<Wall> walls;

    const int num_cols = grid.numCols();
    const double max_thickness_cells = kMaxWallThicknessM * grid.resolution();

    int run_start = -1;  // logical column where the current candidate run started

    for (int col = 0; col < num_cols; ++col)
    {
        const int col_sum = grid.columnOccupancySum(col, col + 1);
        const bool col_is_wall_like = col_sum >= kWallThresholdCells;

        if (col_is_wall_like)
        {
            if (run_start < 0)
            {
                run_start = col;
            }

            // If continuing this run would exceed the max wall thickness,
            // close the current wall here and start a new one at this
            // column. This prevents merging two physically distinct walls
            // that happen to be adjacent with no free gap between them.
            const double current_thickness_cells = static_cast<double>(col - run_start + 1);
            if (current_thickness_cells > max_thickness_cells)
            {
                Wall wall;
                wall.col_start = run_start;
                wall.col_end = col;  // exclusive, i.e. up to (but not including) this column
                wall.thickness_m = static_cast<double>(wall.col_end - wall.col_start) / grid.resolution();
                wall.position_x_m = grid.colToRelX(wall.col_start);
                walls.push_back(wall);

                run_start = col;  // start a new wall from this column
            }
        }
        else if (run_start >= 0)
        {
            // Run ended: close it as a wall.
            Wall wall;
            wall.col_start = run_start;
            wall.col_end = col;
            wall.thickness_m = static_cast<double>(wall.col_end - wall.col_start) / grid.resolution();
            wall.position_x_m = grid.colToRelX(wall.col_start);
            walls.push_back(wall);

            run_start = -1;
        }
    }

    // Close a run that reaches the right edge of the grid.
    if (run_start >= 0)
    {
        Wall wall;
        wall.col_start = run_start;
        wall.col_end = num_cols;
        wall.thickness_m = static_cast<double>(wall.col_end - wall.col_start) / grid.resolution();
        wall.position_x_m = grid.colToRelX(wall.col_start);
        walls.push_back(wall);
    }

    return walls;
}

std::vector<Opening> ObstacleMap::detectOpenings(const OccupancyGrid& grid, const Wall& wall) const
{
    std::vector<Opening> openings;

    const int num_rows = grid.numRows();
    const double min_opening_cells = kMinOpeningSizeM * grid.resolution();

    int run_start = -1;  // row where the current free run started

    for (int row = 0; row < num_rows; ++row)
    {
        const int row_sum = grid.rowOccupancySum(row, row + 1, wall.col_start, wall.col_end);
        const bool row_is_free = (row_sum == 0);

        if (row_is_free)
        {
            if (run_start < 0)
            {
                run_start = row;
            }
        }
        else if (run_start >= 0)
        {
            const double run_height_cells = static_cast<double>(row - run_start);
            if (run_height_cells >= min_opening_cells)
            {
                Opening opening;
                opening.row_start = run_start;
                opening.row_end = row;
                const int center_row = (run_start + row) / 2;
                opening.center_y_m = grid.rowToRelY(center_row);
                openings.push_back(opening);
            }
            run_start = -1;
        }
    }

    // Close a free run that reaches the bottom edge of the grid.
    if (run_start >= 0)
    {
        const double run_height_cells = static_cast<double>(num_rows - run_start);
        if (run_height_cells >= min_opening_cells)
        {
            Opening opening;
            opening.row_start = run_start;
            opening.row_end = num_rows;
            const int center_row = (run_start + num_rows) / 2;
            opening.center_y_m = grid.rowToRelY(center_row);
            openings.push_back(opening);
        }
    }

    return openings;
}

}  // namespace flyappy
