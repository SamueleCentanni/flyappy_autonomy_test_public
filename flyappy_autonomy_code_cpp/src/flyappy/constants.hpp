#pragma once

namespace flyappy
{

// --- Game / sensor constants (from flyappy_main_game) ---
inline constexpr int kNumRays = 9;
inline constexpr double kFovDeg = 90.0;
inline constexpr double kAngleStepDeg = kFovDeg / (kNumRays - 1);
inline constexpr int kCenterRayIdx = (kNumRays - 1) / 2;
inline constexpr double kRangeMax = 3.55;       // meters
inline constexpr double kRangeMaxTol = 0.01;    // meters

inline constexpr double kAccXLimit = 3.0;   // m/s^2
inline constexpr double kAccYLimit = 35.0;  // m/s^2

inline constexpr double kPipeSpacing = 1.92;    // meters, horizontal distance between walls
inline constexpr double kPipeGapSize = 0.5;     // meters, vertical opening height

// --- Bird / body constants ---
inline constexpr double kBirdSize = 0.3;  // meters, used as min opening size & clearance margin

// --- OccupancyGrid constants ---
inline constexpr double kGridResolution = 40.0;  // cells per meter
inline constexpr double kGridWidthM = 6.0;       // meters
inline constexpr double kGridHeightM = 5.5;      // meters

// --- ObstacleMap constants (fixed thresholds, not scaled with resolution) ---
inline constexpr int kWallThresholdCells = 10;      // min occupied cells (summed over wall columns) to call it a wall
inline constexpr double kMaxWallThicknessM = 0.6;   // meters, max thickness before splitting into separate walls
inline constexpr double kMinOpeningSizeM = kBirdSize;  // meters, min opening height to be considered passable

// --- Waypoint / planner constants ---
inline constexpr double kWaypointXMargin = 0.35;  // meters, clearance margin before/after a wall

// --- Controller constants ---
inline constexpr double kTargetVx = 0.5;  // m/s, cruise forward speed

inline constexpr double kKpPosition = 3.0;
inline constexpr double kKiPosition = 0.1;
inline constexpr double kKdPosition = 2.0;
inline constexpr double kIntegralPositionSaturation = 2.0;  // m*s
inline constexpr double kPositionErrorDeadband = 0.05;      // meters

inline constexpr double kKpVelocity = 1.0;

}  // namespace flyappy
