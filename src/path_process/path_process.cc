#include "path_process.h"
#include <memory>
#include "bspline.h"
#include "path_process_api.h"
#include "sampler.h"
// #include "matplotlibcpp.h"
//输出预处理轨迹
path_status ExPathPreprocess(const std::vector<MapPosRecord> &path_in, std::vector<ExPathPoint> &pre_path) {
    // bool res;
    auto res = PathProcess::PathPreprocess(path_in, pre_path);
    return res;
}

//输出平滑轨迹
path_status ExPathsmoothprocess(const std::vector<ExPathPoint> &path_in, const std::vector<BoundaryPoint> &left_boundary,
                                const std::vector<BoundaryPoint> &right_boundary, std::vector<ExPathPoint> &path_out, std::vector<std::vector<xVector2d>> &ALL_intersection) {
    auto smoother = PathProcess::PathSmooth(path_in, left_boundary, right_boundary, path_out, ALL_intersection);
    // return smoother.status();
    return smoother;
}

using planning::xVector2d;
void sample_data(const std::vector<xVector2d> &xys, const double segment_length, std::vector<xVector2d> &xys_sliced) {
    constexpr double kMathEps = 0.0001;
    double kSegmentLength     = segment_length;

    std::vector<double> acc_s;
    acc_s.emplace_back(0);
    for (size_t i = 1; i < xys.size(); i++) {
        const auto &xy      = xys[i];
        const auto &xy_prev = xys[i - 1];
        auto s              = acc_s.back() + std::hypot(xy.x() - xy_prev.x(), xy.y() - xy_prev.y());
        acc_s.emplace_back(s);
    }

    auto total_length  = acc_s.back();
    int segment_size   = std::ceil(total_length / kSegmentLength);
    auto segment_slice = Eigen::VectorXd::LinSpaced(segment_size, 0, total_length);

    xys_sliced.resize(segment_slice.size());
    auto lower = acc_s.begin();
    for (Eigen::Index i = 0; i < segment_slice.size(); i++) {
        auto target_s = segment_slice[i];
        auto upper    = std::upper_bound(lower, acc_s.end(), target_s);

        if (upper == acc_s.end()) {
            xys_sliced[i] = xys.back();
            break;
        }

        auto upper_index = std::distance(acc_s.begin(), upper);
        auto prev_upper  = upper - 1;
        auto den         = *upper - *prev_upper;
        using planning::lerp;
        auto t        = den > kMathEps ? (target_s - *prev_upper) / den : 0;
        auto x        = lerp(xys[upper_index - 1].x(), xys[upper_index].x(), t);
        auto y        = lerp(xys[upper_index - 1].y(), xys[upper_index].y(), t);
        xys_sliced[i] = {x, y};
        lower         = prev_upper;
    }
}

path_status PathProcess::PathPreprocess(const std::vector<MapPosRecord> &path_in, std::vector<ExPathPoint> &pre_paths) {
    // bool res;
    using namespace planning;
    PathSnippetList path_list(path_in);
    auto res_status = path_list.Preprocess();
    if (res_status != VALID) {
        return res_status;
    }

    auto data_point_out = path_list.GetResult();

    // auto sampler_pre     = Sampler(data_point_out);
    // res              = sampler_pre.Sampling();
    // if (!res) {
    //     // status_ = smooth_failed;
    //     return SAMPLING_FAILED;
    // }

    double theta = 0.0;
    double distance = 0.0;
    double dx = 0.0, dy = 0.0;
    for (auto it = data_point_out.begin(); it != data_point_out.end(); ++it) {
        const xVector2d &currentPoint = *it;
        auto nextIt                   = std::next(it);
        const xVector2d &nextPoint    = *nextIt;

        if (nextIt != data_point_out.end()) {
            dx = nextPoint.x() - currentPoint.x();
            dy = nextPoint.y() - currentPoint.y();
            theta = atan2((nextPoint.y() - currentPoint.y()), (nextPoint.x() - currentPoint.x()));
            pre_paths.push_back(ExPathPoint(currentPoint.x(), currentPoint.y(), 0.0, theta, 0.0, distance, 0.0));
            distance += std::sqrt(dx * dx + dy * dy);
        } else {
            pre_paths.push_back(ExPathPoint(currentPoint.x(), currentPoint.y(), 0.0, theta, 0.0, distance, 0.0));
        }
    }
    return VALID;
}

path_status PathProcess::PathSmooth(const std::vector<ExPathPoint> &path_in, const std::vector<BoundaryPoint> &left_boundary,
                                    const std::vector<BoundaryPoint> &right_boundary, std::vector<ExPathPoint> &smooth_path_out, std::vector<std::vector<xVector2d>> &ALL_intersection) {
    bool res;
    using namespace planning;
    smooth_path_out.clear();

    std::vector<BoundaryPoint> obs_comefrom_Semanticline;
    size_t minSize = std::min(left_boundary.size(), right_boundary.size());

    for (size_t i = 0; i < minSize; ++i) {
        obs_comefrom_Semanticline.push_back(BoundaryPoint(left_boundary[i].x, left_boundary[i].y, left_boundary[i].s));
        obs_comefrom_Semanticline.push_back(BoundaryPoint(right_boundary[i].x, right_boundary[i].y, right_boundary[i].s));
    }
    for (size_t i = minSize; i < left_boundary.size(); ++i) {
        obs_comefrom_Semanticline.push_back(BoundaryPoint(left_boundary[i].x, left_boundary[i].y, left_boundary[i].s));
    }
    for (size_t i = minSize; i < right_boundary.size(); ++i) {
        obs_comefrom_Semanticline.push_back(BoundaryPoint(right_boundary[i].x, right_boundary[i].y, right_boundary[i].s));
    }

    std::vector<xVector2d> data_point_in;
    for (const ExPathPoint &it : path_in) {
        data_point_in.push_back(xVector2d(it.x, it.y));
    }

// //------------------------------------------------------------------------
// //记录obs_comefrom_Semanticline_input
// std::ofstream fileobs("/log/hpa_routes/obs_comefrom_Semanticline_input.csv");
// if (!fileobs.is_open()) {
//     std::cerr << "Error opening fileobs: " << "obs_comefrom_Semanticline_input.csv" << std::endl;
//     // return;
// }
// // 写入数据行
// std::size_t min_obs_numb = std::min(right_boundary.size(), right_boundary.size());
// for (size_t i = 0; i < min_obs_numb; i++) {
//     fileobs << right_boundary[i].x;
//     fileobs << ",";
//     fileobs << right_boundary[i].y;
//     fileobs << ",";
//     fileobs << right_boundary[i].s;
//     fileobs << ",";
//     fileobs << left_boundary[i].x;
//     fileobs << ",";
//     fileobs << left_boundary[i].y;
//     fileobs << ",";
//     fileobs << left_boundary[i].s;
//     fileobs << "\n";
// }
// fileobs.close();  

// //记录data_point_in
// std::ofstream filepoint("/log/hpa_routes/data_point_in.csv");
// if (!filepoint.is_open()) {
//     std::cerr << "Error opening filepoint: " << "data_point_in.csv" << std::endl;
//     // return;
// }
// // 写入数据行
// for (size_t i = 0; i < path_in.size(); ++i) {
//     filepoint << path_in[i].x;
//     filepoint << ",";
//     filepoint << path_in[i].y;
//     filepoint << "\n";
// }
// filepoint.close(); 
// //------------------------------------------------------------------------

    std::vector<xVector2d> data_point_new;
    sample_data(data_point_in, 1, data_point_new);

//判断转弯，把直线段拉直
    double Cur;  //判断转弯阈值
    std::vector<int> Corners_Ip;
    std::vector<xVector2d> corners_points;
    for (size_t i = 0; i < data_point_new.size(); i++) {
        // 计算每个点曲率，筛选出转弯点记录ip,设定提前(延后)4个数值点(4m)进入(离开)弯道
        if (i <= 1) {
            Cur = get_curvature(data_point_new[i], data_point_new[i + 2], data_point_new[i + 4]);
        } else if (i >= (data_point_new.size() - 2)) {
            Cur = get_curvature(data_point_new[i], data_point_new[i - 2], data_point_new[i - 4]);
        } else {
            Cur = get_curvature(data_point_new[i], data_point_new[i + 2], data_point_new[i - 2]);
        }
        if (Cur < 50) {  //130
            Corners_Ip.push_back(i);
            if ((i > 1) && (i < data_point_new.size() - 2)) {
                Corners_Ip.push_back(i - 1);
                Corners_Ip.push_back(i - 2);
                Corners_Ip.push_back(i + 1);
                Corners_Ip.push_back(i + 2);
            }
        }
    }
    if(Corners_Ip.empty()){
        std::cout << "The trajectory has no turning" << std::endl;
    }else{
        //删除容器内重复的转弯点ip
        std::unordered_set<int> unique_set;
        Corners_Ip.erase(std::remove_if(Corners_Ip.begin(), Corners_Ip.end(), [&unique_set](int value) {
                            return !unique_set.insert(value).second;
                        }),
                        Corners_Ip.end());
        std::sort(Corners_Ip.begin(), Corners_Ip.end());

        //重构path
        std::vector<xVector2d> Ori_Path_Data = data_point_new;
        //筛选处进入和离开弯道点IP
        std::vector<int> InCorners_PointsIp;
        std::vector<int> OutCorners_PointsIp;
        OutCorners_PointsIp.emplace_back(0);
        InCorners_PointsIp.emplace_back(Corners_Ip.front());
        for (auto ip = Corners_Ip.begin() + 1; ip != Corners_Ip.end() - 1; ++ip) {
            corners_points.emplace_back(Ori_Path_Data[*ip]);
            if (*ip - *(ip - 1) - 1) {
                InCorners_PointsIp.emplace_back(*ip);
            }

            if (*(ip + 1) - *ip - 1) {
                OutCorners_PointsIp.emplace_back(*ip);
            }
        }
        OutCorners_PointsIp.emplace_back(Corners_Ip.back());
        InCorners_PointsIp.emplace_back(static_cast<int>(Ori_Path_Data.size() - 1));

        if (InCorners_PointsIp.size() != OutCorners_PointsIp.size()) {
            std::cout << "Corner Processing Failed" << std::endl;
        }

        // data_point_new.clear();
        // for (size_t i = 0; i < OutCorners_PointsIp.size(); i++) {
        //     if (generateDiscretizedLine(Ori_Path_Data[OutCorners_PointsIp[i]], Ori_Path_Data[InCorners_PointsIp[i]], 2.0, data_point_new)) {
        //         // std::cout << "Trajectory straightened successfully" << std::endl;
        //         if (i == OutCorners_PointsIp.size() - 1) {
        //             break;
        //         }
        //         for (int j = InCorners_PointsIp[i] + 1; j < OutCorners_PointsIp[i + 1]; j++) {
        //             data_point_new.emplace_back(Ori_Path_Data[j]);
        //         }
        //     }
        // }
    }

    auto sampler = Sampler(data_point_new);
    res          = sampler.Sampling();
    if (!res) {
        // status_ = smooth_failed;
        return SMOOTH_FAILED;
    }
    auto control_point = sampler.SamplePoints();

    constexpr int order = 5;
    if (control_point.size() < order) {
        // status_ = too_short;
        return NOT_ENOUGH_CONTROL_POINT;
    }

    if (data_point_new.size() < 3) {
        // status_ = too_short;
        return PREPROCESSED_PATH_TOO_SHORT;
    }
    if (data_point_new.size() > 1000) {
        // status_ = too_long;
        return PREPROCESSED_PATH_TOO_LONG;
    }

    auto smooth_impl = [&ALL_intersection, &control_point, &sampler, &obs_comefrom_Semanticline, &smooth_path_out, &corners_points](BSpline<order> &spline, const std::vector<xVector2d> &data_point_new,
                                                                                                                 std::vector<ExPathPoint> &path_out) -> bool {
        std::vector<double> s_s, x_s, y_s, theta_s, kappa_s, dkappa_s;
        BSplineSmoother<order> smoother(spline, {data_point_new.begin() + 1, data_point_new.end() - 1}, obs_comefrom_Semanticline, corners_points);
        int opt_res = smoother.Optimize(ALL_intersection);

        (void)opt_res;
        auto &opt_spline = smoother.Spline();
        opt_spline.EqualSampleArcLength(1.0, s_s, x_s, y_s, theta_s, kappa_s, dkappa_s);

        for (size_t i = 0; i < s_s.size(); i++) {
            smooth_path_out.emplace_back();
            auto &point = smooth_path_out.back();
            if (fabs(kappa_s[i]) > 0.5) {
                // status_ = kappa_out_range;
                smooth_path_out.clear();
                return false;
            }
            point.x      = x_s[i];
            point.y      = y_s[i];
            point.theta  = theta_s[i];
            point.s      = s_s[i];
            point.kappa  = kappa_s[i];
            point.dkappa = dkappa_s[i];
        }
        // status_ = valid;
        return true;
    };

    auto spline = BSpline<order>(control_point, 1.0e-7, 1.0e-7, 100.0);  //平滑权重 障碍物权重  0.5, 0.5, 10000.0  加拉直：1.0e-7, 1.0e-7, 1.0e-2 不拉直：1.0e-7, 1.0e-7, 1.0e-1
    // auto spline = BSpline<order>(control_point, 0, 0, 0.0);  //0.001, 0.001, 100.0
    if (!smooth_impl(spline, data_point_new, smooth_path_out)) {
        smooth_path_out.clear();
        auto spline2 = BSpline<order>(control_point, 0, 0, 0.0);
        bool succ    = smooth_impl(spline2, data_point_new, smooth_path_out);
        std::cout << "==========================> use second time" << std::endl;
        (void)succ;
        return KAPPA_OUT_OF_RANGE_IN_SMOOTH;
    }
    std::cout << "==========================> use first time" << std::endl;
    return VALID;
    // some additional condition check here
}

std::string Trim(std::string &str) {
    // str.find_first_not_of("
    // \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

double PathProcess::LookForwardDistance(double velocity) {
    auto forward_distance = velocity * FLAGS_look_forward_time_sec;

    return forward_distance > FLAGS_look_forward_short_distance ? FLAGS_look_forward_long_distance
                                                                : FLAGS_look_forward_short_distance;
}
