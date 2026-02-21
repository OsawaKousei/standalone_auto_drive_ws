#pragma once

#include <cstdint>
#include <vector>

namespace ad::types {

struct Point {
  const double x;
  const double y;
};

struct Pose {
  const double x;
  const double y;
  const double theta;
};

struct Twist {
  const double v;
  const double w;
};

struct MapData {
  const int width;
  const int height;
  const double resolution;
  const std::vector<std::int8_t> grid;
};

struct LineSegment {
  const Point start;
  const Point end;
};

struct Footprint {
  const std::vector<Point> vertices;
};

struct LidarScan {
  const std::vector<double> ranges;
  const double minAngle;
  const double angleIncrement;
  const double maxRange;
};

struct OdometryDelta {
  const double deltaForward;
  const double deltaLateral;
  const double deltaTheta;
};

using Path = std::vector<Point>;

} // namespace ad::types
