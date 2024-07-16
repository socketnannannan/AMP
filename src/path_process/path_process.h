#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <future>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "map_io.h"
#include "path_process_api.h"

#pragma GCC system_header

#include <eigen3/Eigen/Eigen>

constexpr double FLAGS_look_backward_distance       = 20;
constexpr double FLAGS_look_forward_short_distance  = 50;
constexpr double FLAGS_look_forward_long_distance   = 100;
constexpr double FLAGS_speed_limit                  = 10;
const double longitudinal_bound_                    = 2.0;
const double interval_                              = 1.0;
const double vehicle_width                          = 2.0;
const double min_lateral_boundary_bound             = 0.2;
const double max_lateral_boundary_bound             = 0.5;
const double lateral_buffer                         = 0.2;
const double FLAGS_look_forward_time_sec            = 16.0;
const double FLAGS_smoothed_reference_line_max_diff = 5.0;

class PathProcess {
public:
    static path_status PathPreprocess(const std::vector<MapPosRecord> &path_in, std::vector<ExPathPoint> &pre_paths);
    static path_status PathSmooth(const std::vector<ExPathPoint> &path_in, const std::vector<BoundaryPoint> &left_boundary,
                                  const std::vector<BoundaryPoint> &right_boundary, std::vector<ExPathPoint> &smooth_path_out, std::vector<std::vector<xVector2d>> &ALL_intersection);
    path_status Checkkappa();
    path_status Checkdkappa();
    path_status Checklength();
    path_status Checkheading();
    path_status Calkappadkappa();
    std::string Trim(std::string &str);
    double LookForwardDistance(double velocity);
    path_status status() const { return status_; }

private:
    path_status status_;
};
