#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include "common.h"

#pragma GCC system_header
#include "RangeTree.h"
#include "emst.hpp"

namespace planning {

namespace RT   = RangeTree;
using RTPoint  = RT::Point<double, int>;
using RTPoints = std::vector<RTPoint>;
using RTTree   = RT::RangeTree<double, int>;
using EMSTTree = EMST::KdTreeSolver<2>;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double>> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

constexpr double H = 5;

struct WeightFunction {
    const xVector2d& P_star{};
    explicit WeightFunction(const xVector2d& P) : P_star(P) {}
    double operator()(const xVector2d& x) {
        auto dx           = x.x() - P_star.x();
        auto dy           = x.y() - P_star.y();
        auto r            = std::hypot(dx, dy);
        auto r_sq         = r * r;
        constexpr auto H3 = H * H * H;
        constexpr auto H2 = H * H;
        auto res          = 2 * r * r_sq / H3 - 3 * r_sq / H2 + 1;
        return res;
    }
};

struct WeightLinearFitting {
    const RTPoints& m_points;
    WeightFunction m_weight_functor;
    double m_a{}, m_b{};
    WeightLinearFitting(const xVector2d& P_star, const RTPoints& points)
        : m_points(points), m_weight_functor(P_star) {}
    void Fitting() {
        double A11{}, A12{}, A21{}, A22{}, b1{}, b2{};
        for (const auto& point : m_points) {
            auto xi      = point.asVector()[0];
            auto yi      = point.asVector()[1];
            auto wi      = m_weight_functor(xVector2d{xi, yi});
            auto wi_x_xi = wi * xi;
            A11 += wi_x_xi * xi;
            A12 += wi_x_xi;
            A21 += wi_x_xi;
            A22 += wi;
            b1 += wi_x_xi * yi;
            b2 += wi * yi;
        }
        auto det   = A11 * A22 - A12 * A21;
        auto det_x = A22 * b1 - A12 * b2;
        auto det_y = A11 * b2 - A21 * b1;
        m_a        = det_x / det;
        m_b        = det_y / det;
    }
};

double compute_curvature_from_3_points(const xVector2d& pose1_, const xVector2d& pose2_, const xVector2d& pose3_) {
    float a = hypot(pose1_.x() - pose2_.x(), pose1_.y() - pose2_.y());
    float b = hypot(pose1_.x() - pose3_.x(), pose1_.y() - pose3_.y());
    float c = hypot(pose3_.x() - pose2_.x(), pose3_.y() - pose2_.y());

    if (fabs(a) < FLT_EPSILON || fabs(b) < FLT_EPSILON || fabs(c) < FLT_EPSILON ||
        fabs(b - a - c) < FLT_EPSILON || fabs(a - b - c) < FLT_EPSILON || fabs(c - a - b) < FLT_EPSILON)
        return 0.0d;

    float k = sqrt((a + b - c) * (a - b + c) * (b + c - a) * (a + b + c)) / (a * b * c);

    return static_cast<double>(k);
}

class Sampler {
    std::shared_ptr<RTTree> rtree_;
    std::shared_ptr<EMSTTree> emstree_;
    const std::vector<xVector2d>& points_;
    std::deque<RT::Point<double, int>> samples_;
    std::vector<xVector2d> samples_result_;
    Graph emst_graph_;
    std::vector<double> emst_graph_distance_;

public:
    explicit Sampler(const std::vector<xVector2d>& points) : points_(points) {
        // construct rt tree
        std::vector<RT::Point<double, int>> rt_points;
        auto f = [](double a, double b) { std::vector<double> d = {a, b}; return d; };
        rt_points.reserve(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            rt_points.emplace_back(f(points[i].x(), points[i].y()), i);
        }
        rtree_ = std::make_shared<RTTree>(rt_points);

        // construct emst tree
        std::vector<EMST::Point<2>> emst_points(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            emst_points[i][0] = points[i].x();
            emst_points[i][1] = points[i].y();
        }
        emstree_             = std::make_shared<EMSTTree>(emst_points);
        const auto& solution = emstree_->get_solution();
        emst_graph_          = Graph(emst_points.size());
        for (const auto& edge : solution) {
            auto idx_a = edge.first;
            auto idx_b = edge.second;
            auto dx    = points[idx_a].x() - points[idx_b].x();
            auto dy    = points[idx_a].y() - points[idx_b].y();
            boost::add_edge(idx_a, idx_b, std::hypot(dx, dy), emst_graph_);
        }
        emst_graph_distance_.resize(boost::num_vertices(emst_graph_));
    }

    const std::vector<xVector2d>& SamplePoints() const {
        return samples_result_;
    }

    void SampleData(const std::vector<xVector2d>& xys, const double segment_length, std::vector<xVector2d>& xys_sliced) {
        //        double kMathEps = 0.0001;
        double kSegmentLength = segment_length;

        std::vector<double> acc_s;
        acc_s.emplace_back(0);
        for (size_t i = 1; i < xys.size(); i++) {
            const auto& xy      = xys[i];
            const auto& xy_prev = xys[i - 1];
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

    bool Sampling() {
        // const auto gather_points = [this](const RTPoint& middle, RTPoints& points) {
        //     auto xy_middle = xVector2d{middle.asVector()[0], middle.asVector()[1]};
        //     std::vector<double> lower{xy_middle.x() - H, xy_middle.y() - H};
        //     std::vector<double> upper{xy_middle.x() + H, xy_middle.y() + H};
        //     std::vector<bool> withLower{false, false};
        //     std::vector<bool> withUpper{false, false};
        //     points = rtree_->pointsInRange(lower, upper, withLower, withUpper);

        //     // filter two far distance vertex in EMST
        //     boost::dijkstra_shortest_paths(emst_graph_, middle.value(),
        //                                    boost::distance_map(boost::make_iterator_property_map(
        //                                        emst_graph_distance_.begin(), boost::get(boost::vertex_index, emst_graph_))));
        //     auto remove_it = std::remove_if(points.begin(), points.end(), [this](const RTPoint& x) {
        //         return this->emst_graph_distance_[x.value()] > 10;
        //     });
        //     points.erase(remove_it, points.end());
        // };

        // const auto far_element_in_direction =
        //     [](const RTPoints& points, const xVector2d& origin, const xVector2d& direction) {
        //         auto far_element =
        //             std::max_element(points.begin(), points.end(),
        //                              [&](const RTPoint& a, const RTPoint& b) {
        //                                  return direction.dot({a.asVector()[0] - origin.x(), a.asVector()[1] - origin.y()}) <
        //                                         direction.dot({b.asVector()[0] - origin.x(), b.asVector()[1] - origin.y()});
        //                              });
        //         return far_element;
        //     };

        // const auto sample_once =
        //     [&gather_points, &far_element_in_direction](const RTPoint& middle, xVector2d& ref_vector, RTPoint& res) -> bool {
        //     auto xy_middle = xVector2d{middle.asVector()[0], middle.asVector()[1]};
        //     RTPoints points;
        //     gather_points(middle, points);
        //     auto fitter = WeightLinearFitting(xy_middle, points);
        //     fitter.Fitting();
        //     xVector2d vec = {1, fitter.m_a};
        //     if (vec.dot(ref_vector) < 0) {
        //         vec = {-1, -fitter.m_a};
        //     }
        //     auto far_element = far_element_in_direction(points, xy_middle, vec);

        //     res = *far_element;
        //     auto xy_res = xVector2d{far_element->asVector()[0], far_element->asVector()[1]};
        //     ref_vector = vec;
        //     if (vec.dot(xy_res - xy_middle) < kMathEps)
        //         return false;

        //     return true;
        // };

        // auto N = points_.size();
        // auto middle = points_[N / 2];

        // auto f = [](double a, double b) { std::vector<double> d = {a, b}; return d; };
        // samples_.emplace_back(f(middle.x(), middle.y()), N / 2);
        // RTPoints points;
        // gather_points(samples_.back(), points);
        // auto fitter = WeightLinearFitting(middle, points);
        // fitter.Fitting();

        // // sample along side a
        // xVector2d ref_vec_a = {1, fitter.m_a};
        // RTPoint res_a;
        // auto a_element = far_element_in_direction(points, middle, ref_vec_a);
        // samples_.emplace_back(*a_element);
        // while (sample_once(samples_.back(), ref_vec_a, res_a)) {
        //     samples_.emplace_back(res_a);
        // }

        // // sample along side b
        // xVector2d ref_vec_b = {-1, -fitter.m_a};
        // RTPoint res_b;
        // auto b_element = far_element_in_direction(points, middle, ref_vec_b);
        // samples_.emplace_front(*b_element);
        // while (sample_once(samples_.front(), ref_vec_b, res_b)) {
        //     samples_.emplace_front(res_b);
        // }

        // // by condition samples should be reverse
        // auto a_vector = points_.front() - xVector2d{samples_.front().asVector()[0], samples_.front().asVector()[1]};
        // auto b_vector = points_.back() - xVector2d{samples_.front().asVector()[0], samples_.front().asVector()[1]};
        // if (a_vector.squaredNorm() > b_vector.squaredNorm()) {
        //     std::reverse(samples_.begin(), samples_.end());
        // }

        // bool odd_size = samples_.size() % 2;
        // for (size_t i = 0; i < samples_.size(); i += 2) {
        //     samples_result_.emplace_back(samples_[i].asVector()[0], samples_[i].asVector()[1]);
        // }

        // if (not samples_.empty() and not odd_size) {
        //     samples_result_.emplace_back(samples_.back().asVector()[0], samples_.back().asVector()[1]);
        // }

        // return !samples_result_.empty();

        // int segment_seq = 0;
        // std::vector<xVector2d> data_point_bk;
        // bool bad_start                 = true;
        // int index                      = 0;
        // double distance                = 0;
        // double acceptable_start_length = 3;
        // int start_index                = 0;
        double sample_resolution = 3;
        // while ((index + 2) < (int)points_.size()) {
        //     if (distance >= acceptable_start_length) {
        //         bad_start = false;
        //         break;
        //     }
        //     xVector2d first2second(points_[index + 1] - points_[index]);
        //     xVector2d second2third(points_[index + 2] - points_[index + 1]);
        //     distance += hypot(points_[index + 1].x() - points_[index].x(),
        //                       points_[index + 1].y() - points_[index].y());
        //     if (second2third.dot(first2second) < 0) {
        //         start_index = index + 1;
        //         distance    = 0;
        //     }
        //     index++;
        // }
        // if (bad_start)
        //     return false;
        // auto res1 = std::find_if(points_.begin(), points_.end(),
        //                          [&data_point_bk, &segment_seq, &start_index](const xVector2d& cur_point) {
        //                              if (segment_seq < start_index) {
        //                                  segment_seq++;
        //                                  return false;
        //                              }
        //                              if (segment_seq <= start_index + 1) {
        //                                  data_point_bk.emplace_back(cur_point);
        //                              } else {
        //                                  xVector2d last_dir = data_point_bk[data_point_bk.size() - 1] - data_point_bk[data_point_bk.size() - 2];
        //                                  xVector2d cur_dir  = cur_point - data_point_bk[data_point_bk.size() - 1];
        //                                  if (last_dir.dot(cur_dir) >= 0)
        //                                      data_point_bk.emplace_back(cur_point);
        //                              }
        //                              segment_seq++;
        //                              return false;
        //                          });
        // (void)res1;
        SampleData(points_, sample_resolution, samples_result_);
        if (hypot(samples_result_.back().x() - points_.back().x(), samples_result_.back().y() - points_.back().y()) > kMathEps)
            samples_result_.emplace_back(points_.back());
        return !samples_result_.empty();
    }
};

class PathSnippet {
    std::vector<xVector2d> xys_;
    std::vector<double> kappas_, accumulated_s_;
    int direction_;

public:
    PathSnippet(const std::vector<xVector2d>& xys, int direction)
        : xys_(xys), direction_(direction) {
        ComputePathProfile();
    }

    std::vector<xVector2d>& xys() { return xys_; }
    std::vector<double>& kappas() { return kappas_; }
    std::vector<double>& accumulated_s() { return accumulated_s_; }

    int Direction() const { return direction_; }

    double Length() const { return accumulated_s_.back(); }

    // double Project(const xVector2d& xy) const {
    std::pair<size_t, double> Project(const xVector2d& xy) const {
        double min_distance = std::numeric_limits<double>::infinity();
        double min_distance_sign;
        size_t idx = 0;
        for (size_t i = 0; i < xys_.size() - 1; i++) {
            auto dx = xys_[i + 1].x() - xys_[i].x();
            auto dy = xys_[i + 1].y() - xys_[i].y();
            xVector2d n(-dy, dx);
            dx                = xy.x() - xys_[i].x();
            dy                = xy.y() - xys_[i].y();
            auto distance     = n.dot({dx, dy}) / n.norm();
            auto abs_distance = std::fabs(distance);
            if (abs_distance < min_distance) {
                min_distance      = abs_distance;
                min_distance_sign = distance;
                idx               = i;
            }
        }
        return std::make_pair(idx, min_distance);
    }

    double AverageKappa() const {
        auto kappa_sum = std::accumulate(kappas_.begin(), kappas_.end(), 0.0, [](double a, double b) {
            return fabs(a) + b;
        });
        return kappa_sum / static_cast<double>(kappas_.size());
    }

    bool ComputePathProfile() {
        double distance = 0.0;
        accumulated_s_.push_back(distance);
        if (xys_.size() < 2) {
            return false;
        }
        auto len = xys_.size();
        std::vector<double> dxs;
        dxs.reserve(len);
        std::vector<double> dys;
        dys.reserve(len);
        std::vector<double> y_over_s_first_derivatives;
        y_over_s_first_derivatives.reserve(len);
        std::vector<double> x_over_s_first_derivatives;
        x_over_s_first_derivatives.reserve(len);
        std::vector<double> y_over_s_second_derivatives;
        y_over_s_second_derivatives.reserve(len);
        std::vector<double> x_over_s_second_derivatives;
        x_over_s_second_derivatives.reserve(len);

        // Get finite difference approximated dx and dy for heading and kappa
        // calculation
        std::size_t points_size = xys_.size();
        for (std::size_t i = 0; i < points_size; ++i) {
            double x_delta = 0.0;
            double y_delta = 0.0;
            if (i == 0) {
                x_delta = (xys_[i + 1].x() - xys_[i].x());
                y_delta = (xys_[i + 1].y() - xys_[i].y());
            } else if (i == points_size - 1) {
                x_delta = (xys_[i].x() - xys_[i - 1].x());
                y_delta = (xys_[i].y() - xys_[i - 1].y());
            } else {
                x_delta = 0.5 * (xys_[i + 1].x() - xys_[i - 1].x());
                y_delta = 0.5 * (xys_[i + 1].y() - xys_[i - 1].x());
            }
            dxs.push_back(x_delta);
            dys.push_back(y_delta);
        }

        // Get linear interpolated s for dkappa calculation
        double fx = xys_[0].x();
        double fy = xys_[0].y();
        double nx = 0.0;
        double ny = 0.0;
        for (std::size_t i = 1; i < points_size; ++i) {
            nx = xys_[i].x();
            ny = xys_[i].y();
            double end_segment_s =
                std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
            accumulated_s_.push_back(end_segment_s + distance);
            distance += end_segment_s;
            fx = nx;
            fy = ny;
        }

        double kappa       = 0.0;
        double kappa_start = compute_curvature_from_3_points(xys_[0], xys_[1], xys_[2]);
        kappas_.push_back(kappa_start);
        for (std::size_t i = 1; i < points_size; ++i) {
            if (i == points_size - 1) {
                kappas_.push_back(kappa);
                break;
            }
            kappa = compute_curvature_from_3_points(xys_[i - 1], xys_[i], xys_[i + 1]);
            kappas_.push_back(kappa);
        }
        return true;
    }
};

class PathSnippetList {
    std::vector<PathSnippet> path_list_;
    std::vector<xVector2d> path_result_;
    path_status status_;

public:
    path_status status() const { return status_; }

    explicit PathSnippetList(const std::vector<MapPosRecord>& path_record) {
        std::vector<MapPosRecord> tmp_path;
        std::copy_if(path_record.begin(), path_record.end(), std::back_inserter(tmp_path), [](const MapPosRecord& point) {
            return point.direction != 0;
        });
        int direction = tmp_path.front().direction;
        std::vector<xVector2d> xys;
        for (const auto& point : tmp_path) {
            if (point.direction != direction) {
                path_list_.emplace_back(xys, direction);
                xys.clear();
                xys.emplace_back(point.pose.P[0], point.pose.P[1]);
                direction = point.direction;
                continue;
            }
            xys.emplace_back(point.pose.P[0], point.pose.P[1]);
        }
        path_list_.emplace_back(xys, direction);
    }

    path_status Preprocess() {
        auto res_1 = SkipTerminalSnippet();
        if (res_1 != VALID) return res_1;
        bool res{};
        res = HandleReverseSnippet();
        if (!res) return HANDLE_REVERSE_FAILED;
        res = CheckAverageKappa();
        if (!res) return KAPPA_OUT_OF_RANGE_IN_PREPROCESS;
        for (auto& snippet : path_list_) {
            auto& xys = snippet.xys();
            std::copy(xys.begin(), xys.end(), std::back_inserter(path_result_));
        }
        return VALID;
    }

    const std::vector<xVector2d>& GetResult() const { return path_result_; }

    path_status SkipTerminalSnippet() {
        path_list_.erase(
            std::remove_if(path_list_.begin(), path_list_.end(),
                           [](const PathSnippet& snippet) { return snippet.Length() < 0.5; }),
            path_list_.end());
        auto start_snippet =
            std::find_if(path_list_.begin(), path_list_.end(),
                         [](const PathSnippet& snippet) { return snippet.Direction() == 1 && snippet.Length() > 5; });
        if (start_snippet == path_list_.end()) {
            return CANNOT_FIND_START_SNIPPET;
        }

        xVector2d parked_xy               = path_list_.back().xys().back();
        size_t end_snippet_idx            = path_list_.size();
        int end_path_point_idx_in_snippet = -1;
        for (size_t i = 0; i < path_list_.size(); ++i) {
            if (path_list_.at(i).Direction() != 1) {
                continue;
            }
            for (size_t j = 0; j < path_list_.at(i).xys().size(); ++j) {
                auto xy              = path_list_.at(i).xys().at(j);
                double tmp_manh_dist = std::fabs(xy.x() - parked_xy.x()) +
                                       std::fabs(xy.y() - parked_xy.y());
                if (tmp_manh_dist < 10.0) {
                    end_path_point_idx_in_snippet = static_cast<int>(j);
                    break;
                }
            }
            if (end_path_point_idx_in_snippet != -1) {
                end_snippet_idx = i;
                break;
            }
        }
        if (end_snippet_idx == path_list_.size()) {
            return CANNOT_FIND_END_SNIPPET;
        } else if (end_path_point_idx_in_snippet == -1) {
            return CANNOT_FIND_END_SNIPPET;
        }

        auto finish_snippet = path_list_.begin();
        std::advance(finish_snippet, end_snippet_idx);
        path_list_.assign(start_snippet, finish_snippet + 1);
        return VALID;
    }

    bool HandleReverseSnippet() {
        static const double lat_dist_threshold = 2.0;
        for (size_t i = 0; i < path_list_.size(); ++i) {
            if (path_list_.at(i).Direction() == -1) {
                // check projection between i-1 and i+1 path
                if (i < path_list_.size() - 1) {
                    auto& path_a      = path_list_.at(i - 1);
                    auto& path_b      = path_list_.at(i + 1);
                    auto a_end_to_b   = path_b.Project(path_a.xys().back());
                    auto b_start_to_a = path_a.Project(path_b.xys().front());
                    int type          = 0;  // 0: a_end_to_b, 1: b_start_to_a
                    auto min_pair     = a_end_to_b;
                    if (a_end_to_b.second > b_start_to_a.second) {
                        auto min_pair = b_start_to_a;
                        type          = 1;
                    }
                    if (min_pair.second < lat_dist_threshold) {
                        if (type == 0) {
                            path_b.xys().erase(path_b.xys().begin(),
                                               path_b.xys().begin() + min_pair.first);
                            path_b.kappas().erase(path_b.kappas().begin(),
                                                  path_b.kappas().begin() + min_pair.first);
                            path_b.accumulated_s().erase(path_b.accumulated_s().begin(),
                                                         path_b.accumulated_s().begin() + min_pair.first);
                        } else if (type == 1) {
                            path_a.xys().erase(path_a.xys().begin() + min_pair.first + 1,
                                               path_a.xys().end());
                            path_a.kappas().erase(path_a.kappas().begin() + min_pair.first + 1,
                                                  path_a.kappas().end());
                            path_a.accumulated_s().erase(path_a.accumulated_s().begin() + min_pair.first + 1,
                                                         path_a.accumulated_s().end());
                        }
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
        auto reverse_snippet = std::remove_if(path_list_.begin(), path_list_.end(),
                                              [](const PathSnippet& snippet) { return snippet.Direction() == -1; });
        path_list_.erase(reverse_snippet, path_list_.end());
        return true;
    }

    bool CheckAverageKappa() {
        auto res = std::all_of(path_list_.begin(), path_list_.end(), [](const PathSnippet& snippet) {
            return snippet.AverageKappa() < 0.2;
        });
        return res;
    }
};

}  // namespace planning
