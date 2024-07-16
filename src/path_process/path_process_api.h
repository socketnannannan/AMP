
#pragma once

#include <string>
#include "common.h"
#include "map_io.h"

// path_status ExPathprocess(const std::vector<MapPosRecord>& path_in, std::vector<ExPathPoint>& path_out);
using planning::xVector2d;
path_status ExPathPreprocess(const std::vector<MapPosRecord> &path_in, std::vector<ExPathPoint> &pre_path);
path_status ExPathsmoothprocess(const std::vector<ExPathPoint> &path_in, const std::vector<BoundaryPoint> &left_boundary,
                                const std::vector<BoundaryPoint> &right_boundary, std::vector<ExPathPoint> &path_out,  std::vector<std::vector<xVector2d>> &ALL_intersection);