#pragma once

#include <algorithm>
#include <boost/math/tools/roots.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <unordered_set>
#include <vector>
#include "Large_Conve.h"
#include "common.h"
#include "lbfgs.hpp"
#include "write_data_in_csv.h"
// #include "read_csv_data.h"

//得到直线系数A b
void get_Ab(const xVector2d &A, const xVector2d &B, Eigen::MatrixXd &A_s, double &b_s) {
    double A_coeff = 0.0, B_coeff = 0.0, C_coeff = 0.0;

    // 计算直线的系数 A, B, C   Ax+By+C=0
    if (!(A == B)) {
        A_coeff = B.y() - A.y();
        B_coeff = A.x() - B.x();
        C_coeff = A.y() * B.x() - A.x() * B.y();
    } else {
        std::cout << "have something false in function: Large_Conve()" << std::endl;
    }
    A_s(0, 0) = A_coeff;
    A_s(0, 1) = B_coeff;
    b_s       = C_coeff;
}

//得到多边形系数A b
void get_safe_area(std::vector<bool>& optimization_flags, std::vector<xVector2d>& obstest, const std::vector<BoundaryPoint> &all_obs, const xVector2d &point, const double local_s,
     const double &safe_R, Eigen::MatrixXd &A, Eigen::VectorXd &b, std::vector<int> &allpoint_poly_numes, std::vector<std::vector<xVector2d>> &ALL_intersection123) {
    //筛选半径7内障碍物且取本车点周围200个障碍物点（解决十字路口障碍物侵入问题）
    static const double obs_range= 7.0;
    std::vector<xVector2d> obs_point;
    for (auto it = all_obs.begin(); it != all_obs.end(); ++it) {
        const BoundaryPoint &currentObs = *it;
        if (std::fabs(currentObs.s - local_s) < obs_range) {
                obs_point.push_back(xVector2d(currentObs.x, currentObs.y));
        }
        // if ((currentObs.x - point.x()) * (currentObs.x - point.x()) +
        //         (currentObs.y - point.y()) * (currentObs.y - point.y()) <
        //     obs_range) {
        //         obs_point.push_back(xVector2d(currentObs.x, currentObs.y));
        // }
    }

    int onepoint_poly_numes = 0;
    Eigen::MatrixXd A_s(1, 2);
    double b_s;

    obstest = obs_point;
    //得到多边形交点
    std::vector<xVector2d> vertex_point = Large_Conve(point, obs_point, safe_R);

    if(vertex_point.empty()){
        bool optimization_flag = false;
        optimization_flags.emplace_back(optimization_flag);
    }else{
        optimization_flags.emplace_back(true);
    }
    ALL_intersection123.emplace_back(vertex_point);

    //存储每个点对应凸包的各个边的参数和边数
    for (auto it = vertex_point.begin(); it != vertex_point.end(); ++it) {
        const xVector2d &currentPoint = *it;
        auto nextIt                   = std::next(it);
        if (nextIt == vertex_point.end()) {
            nextIt = vertex_point.begin();
        }
        const xVector2d &nextPoint = *nextIt;

        get_Ab(currentPoint, nextPoint, A_s, b_s);
        A(onepoint_poly_numes, 0) = A_s(0, 0);
        A(onepoint_poly_numes, 1) = A_s(0, 1);
        b(onepoint_poly_numes)    = b_s;
        onepoint_poly_numes++;
    }
    allpoint_poly_numes.push_back(onepoint_poly_numes);
}

//计算曲率参数
double get_curvature(const xVector2d &p0, const xVector2d &pf, const xVector2d &pr) {
    Eigen::MatrixXd A(2, 2);
    Eigen::VectorXd B(2);

    A << 2 * (p0.x() - pf.x()), 2 * (p0.y() - pf.y()),
        2 * (pr.x() - p0.x()), 2 * (pr.y() - p0.y());

    B << (p0.x() * p0.x() + p0.y() * p0.y()) - (pf.x() * pf.x() + pf.y() * pf.y()),
        (pr.x() * pr.x() + pr.y() * pr.y()) - (p0.x() * p0.x() + p0.y() * p0.y());

    Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);
    double R2              = (p0.x() - result(0)) * (p0.x() - result(0)) + (p0.y() - result(1)) * (p0.y() - result(1));
    if (R2 > 1000) {
        R2 = 500;
    }
    return R2;
}

//对直线离散采样
bool generateDiscretizedLine(const xVector2d &start_point, const xVector2d &end_point, double step, std::vector<xVector2d> &data_points) {
    double distance = std::hypot(end_point.x() - start_point.x(), end_point.y() - start_point.y());
    if (distance == 0) {
        data_points.emplace_back(start_point);
        std::cout << "Turn at the beginning or end of the trajectory" << std::endl;
        return false;
    }
    double dir_x   = (end_point.x() - start_point.x()) / distance;
    double dir_y   = (end_point.y() - start_point.y()) / distance;
    int num_points = static_cast<int>(std::ceil(distance / step));

    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = start_point.x() + t * distance * dir_x;
        double y = start_point.y() + t * distance * dir_y;
        data_points.emplace_back(xVector2d(x, y));
    }
    return true;
}

struct arc_length_functor {
    explicit arc_length_functor(const std::function<double(double, double &)> &func) : func_(func) {}

    boost::math::tuple<double, double> operator()(double const &x) {
        double g;
        auto f = func_(x, g);
        return boost::math::make_tuple(f, g);
    }

private:
    std::function<double(double, double &)> func_;
};

namespace planning {

constexpr double kMathEps = 1e-5;

template <int Order>
class Polynomial {
    double coef_[Order]        = {};
    double d_coef_[Order - 1]  = {};
    double d2_coef_[Order - 2] = {};
    double d3_coef_[Order - 3] = {};

public:
    Polynomial() = default;

    explicit Polynomial(double coef[Order]) {
        memcpy(&coef_[0], &coef[0], sizeof(double) * Order);
        for (int i = 0; i < Order - 1; i++) {
            d_coef_[i] = coef_[i + 1] * (i + 1);
        }
        for (int i = 0; i < Order - 2; i++) {
            d2_coef_[i] = d_coef_[i + 1] * (i + 1);
        }
        for (int i = 0; i < Order - 3; i++) {
            d3_coef_[i] = d2_coef_[i + 1] * (i + 1);
        }
    }

    Polynomial(std::initializer_list<double> coef) {
        std::copy(std::begin(coef), std::end(coef), std::begin(coef_));
        for (int i = 0; i < Order - 1; i++) {
            d_coef_[i] = coef_[i + 1] * (i + 1);
        }
        for (int i = 0; i < Order - 2; i++) {
            d2_coef_[i] = d_coef_[i + 1] * (i + 1);
        }
        for (int i = 0; i < Order - 3; i++) {
            d3_coef_[i] = d2_coef_[i + 1] * (i + 1);
        }
    }

    double Coef(const size_t order) const {
        return coef_[order];
    }

    double dCoef(const size_t order) const {
        return d_coef_[order];
    }

    double d2Coef(const size_t order) const {
        return d2_coef_[order];
    }

    double *Coefs() const {
        return (double *)coef_;
    }

    double *dCoefs() const {
        return (double *)d_coef_;
    }

    double *d2Coefs() const {
        return (double *)d2_coef_;
    }

    double d3Coef(const size_t order) const {
        return d3_coef_[order];
    }

    double Evaluate(const std::uint32_t order, const double p) const {
        double res;
        switch (order) {
            case 0: {
                res = coef_[Order - 1];
                for (int i = 0; i < Order - 1; i++) {
                    res = res * p + coef_[Order - 2 - i];
                }
                break;
            }
            case 1: {
                res = d_coef_[Order - 2];
                for (int i = 0; i < Order - 2; i++) {
                    res = res * p + d_coef_[Order - 3 - i];
                }
                break;
            }
            case 2: {
                res = d2_coef_[Order - 3];
                for (int i = 0; i < Order - 3; i++) {
                    res = res * p + d2_coef_[Order - 4 - i];
                }
                break;
            }
            case 3: {
                res = d3_coef_[Order - 4];
                for (int i = 0; i < Order - 4; i++) {
                    res = res * p + d3_coef_[Order - 5 - i];
                }
                break;
            }
            default:
                return 0.0;
        }
        return res;
    }

    Polynomial &operator+=(const Polynomial<Order> &other) {
        for (int i = 0; i < Order; i++) {
            coef_[i] += other.coef_[i];
        }
        for (int i = 0; i < Order - 1; i++) {
            d_coef_[i] += other.d_coef_[i];
        }
        for (int i = 0; i < Order - 2; i++) {
            d2_coef_[i] += other.d2_coef_[i];
        }
        for (int i = 0; i < Order - 3; i++) {
            d3_coef_[i] += other.d3_coef_[i];
        }
        return *this;
    }

    Polynomial &operator*(const double k) {
        for (int i = 0; i < Order; i++) {
            coef_[i] *= k;
        }
        for (int i = 0; i < Order - 1; i++) {
            d_coef_[i] *= k;
        }
        for (int i = 0; i < Order - 2; i++) {
            d2_coef_[i] *= k;
        }
        for (int i = 0; i < Order - 3; i++) {
            d3_coef_[i] *= k;
        }
        return *this;
    }

    Polynomial &operator/(const double k) {
        for (int i = 0; i < Order; i++) {
            coef_[i] /= k;
        }
        for (int i = 0; i < Order - 1; i++) {
            d_coef_[i] /= k;
        }
        for (int i = 0; i < Order - 2; i++) {
            d2_coef_[i] *= k;
        }
        for (int i = 0; i < Order - 3; i++) {
            d3_coef_[i] *= k;
        }
        return *this;
    }

    double Integral(double lb, double ub) {
        double coef[Order + 1] = {};
        for (int i = 0; i < Order; i++) {
            coef[i + 1] = 1.0 / (i + 1) * coef_[i];
        }
        Polynomial<Order + 1> poly(coef);
        return poly.Evaluate(0, ub) - poly.Evaluate(0, lb);
    }

    template <int Order1, int Order2>
    friend Polynomial<Order1 + Order2 - 1> operator*(const Polynomial<Order1> &a, const Polynomial<Order2> &b);

    template <int xOrder>
    friend Polynomial<xOrder> operator*(const Polynomial<xOrder> &a, double k);

    template <int Order1, int Order2>
    friend Polynomial<std::max(Order1, Order2)> operator+(const Polynomial<Order1> &a, const Polynomial<Order2> &b);

    template <int xOrder>
    Polynomial<xOrder> Truncate() {
        double coef[xOrder]{};
        memcpy(&coef[0], &coef_[0], sizeof(double) * xOrder);
        return Polynomial<xOrder>(coef);
    }
};

template <int xOrder>
Polynomial<xOrder> operator*(const Polynomial<xOrder> &a, double k) {
    Polynomial<xOrder> res = a;
    return res * k;
}

template <int Order1, int Order2>
Polynomial<Order1 + Order2 - 1> operator*(const Polynomial<Order1> &a, const Polynomial<Order2> &b) {
    double coef[Order1 + Order2 - 1] = {};
    for (int i = 0; i < Order1; i++) {
        for (int j = 0; j < Order2; j++) {
            int n = i + j;
            coef[n] += a.coef_[i] * b.coef_[j];
        }
    }
    return Polynomial<Order1 + Order2 - 1>(coef);
}

template <int Order1, int Order2>
Polynomial<std::max(Order1, Order2)> operator+(const Polynomial<Order1> &a, const Polynomial<Order2> &b) {
    double coef[std::max(Order1, Order2)] = {};

    for (int i = 0; i < std::max(Order1, Order2); i++) {
        double ax{}, bx{};
        if (i < Order1)
            ax = a.coef_[i];
        if (i < Order2)
            bx = b.coef_[i];
        coef[i] = ax + bx;
    }
    return Polynomial<std::max(Order1, Order2)>(coef);
}

template <int Order>
class BSplineBasis {
    double knots_[Order + 1]{};
    Polynomial<Order> polys_[Order]{};

public:
    explicit BSplineBasis(double knots[Order + 1]) {
        memcpy(&knots_[0], &knots[0], sizeof(double) * (Order + 1));

        for (int interval_index = 0; interval_index < Order; interval_index++) {
            Polynomial<Order> N_x_n_prev[Order]{}, N_x_n[Order]{};

            // for order 1 case
            // for t in interval [knots[i], knots[i+1])
            if (knots[interval_index] != knots[interval_index + 1]) {
                // interval length is not zeros
                N_x_n_prev[interval_index] = {1, 0};
            } else {
                // interval length is zeros, and other order 1 term must be zero too
                // since we in interval [knots[i], knots[i+1])
                polys_[interval_index] = {0};
                continue;
            }

            // for order 1 above
            for (int order_index = 1; order_index < Order; order_index++) {
                auto effective_basis_number = Order - order_index;
                for (int basis_index = 0; basis_index < effective_basis_number; basis_index++) {
                    /*
                     *                t - t_i             t_i+k - t
                     *  N_i_k = ------------- * N_i_k-1 + ------------- * N_i+1_k-1
                     *          t_i+k-1 - t_i             t_i+k - t_i+1
                     *
                     *  t_i <- t_i          t_i+1 <- t_ip1
                     *  t_i+k-1 <- t_ipkm1  t_i+k <- t_ipk
                     */
                    auto k   = order_index + 1;
                    auto t_i = knots_[basis_index], t_ip1 = knots_[basis_index + 1],
                         t_ipkm1 = knots_[basis_index + k - 1], t_ipk = knots_[basis_index + k];
                    auto den_a = t_ipkm1 - t_i;
                    auto den_b = t_ipk - t_ip1;

                    Polynomial<Order> poly_a = den_a > 0 ? (Polynomial<Order>({-t_i, 1}) / den_a *
                                                            N_x_n_prev[basis_index])
                                                               .template Truncate<Order>()
                                                         : Polynomial<Order>({0});
                    Polynomial<Order> poly_b = den_b > 0 ? (Polynomial<Order>({t_ipk, -1}) / den_b *
                                                            N_x_n_prev[basis_index + 1])
                                                               .template Truncate<Order>()
                                                         : Polynomial<Order>({0});
                    N_x_n[basis_index] = poly_a + poly_b;
                }

                // overwrite N_x_n_prev by N_x_n
                for (int basis_index = 0; basis_index < effective_basis_number; basis_index++) {
                    N_x_n_prev[basis_index] = N_x_n[basis_index];
                }
            }
            polys_[interval_index] = N_x_n[0];
        }
    }

    double Evaluate(const std::uint32_t order, const double p) const {
        if (p < knots_[0] or p > knots_[Order])
            return 0;

        auto upper = std::upper_bound(std::begin(knots_), std::end(knots_), p);
        if (upper == std::end(knots_)) {
            upper = std::lower_bound(std::begin(knots_), std::end(knots_), p);
        }
        auto &poly = polys_[upper - &knots_[0] - 1];

        return poly.Evaluate(order, p);
    }

    const Polynomial<Order> *GetPolynomial() const { return polys_; }
};

template <int Order>
class BSpline {
    double alpha = 1e-7, beta = 1e-7, obstacle_Weights_ = 10000.0;
    using MatrixNd = Eigen::Matrix<double, Order, Order>;

    size_t control_points_size_;
    std::vector<xVector2d> control_points_;
    std::vector<xVector2d> control_points_grad_;
    size_t knots_size_;
    std::vector<double> knots_;
    size_t spans_size_;
    std::vector<BSplineBasis<Order>> basis_;
    std::vector<Polynomial<Order>> weighted_basis_x_;
    std::vector<Polynomial<Order>> weighted_basis_y_;
    std::vector<MatrixNd> order1_smooth_cost_coeff_;
    std::vector<MatrixNd> order2_smooth_cost_coeff_;
    xVector2d start_velocity_, end_velocity_;
    double start_grad_[Order] = {}, end_grad_[Order] = {};

    double DistanceEvaluate(const xVector2d &point, double t, double &grad_t) {
        constexpr double grad_offset[Order] = {1, 0, 0, 0};
        if (t <= 0) {
            xVector2d foot_point = control_points_.front() + start_velocity_ * (t - 0);
            xVector2d vec        = foot_point - point;
            grad_t               = 2 * vec.dot(start_velocity_);
            auto f_it_grad       = control_points_grad_.begin();
            for (auto i = 0; i < Order; i++, f_it_grad++) {
                *f_it_grad += 2 * vec * (start_grad_[i] * (t - 0) + grad_offset[i]);
            }
            return vec.squaredNorm();
        } else if (t >= 1) {
            xVector2d foot_point = control_points_.back() + end_velocity_ * (t - 1);
            xVector2d vec        = foot_point - point;
            grad_t               = 2 * vec.dot(end_velocity_);
            auto r_it_grad       = control_points_grad_.rbegin();
            for (auto i = 0; i < Order; i++, r_it_grad++) {
                *r_it_grad += 2 * vec * (end_grad_[i] * (t - 1) + grad_offset[i]);
            }
            return vec.squaredNorm();
        }

        auto upper         = std::upper_bound(knots_.begin(), knots_.end(), t);
        size_t index_start = std::distance(knots_.begin(), upper) - Order;
        if (upper == knots_.end()) {
            index_start = control_points_size_ - Order;
        }

        xVector2d foot_point(0, 0);
        double grad_basis_array[Order];
        xVector2d grad_t_temp(0, 0);
        for (int i = 0; i < Order; i++) {
            const auto &control_point = control_points_[index_start + i];
            const auto &basis         = basis_[index_start + i];
            auto val                  = basis.Evaluate(0, t);
            foot_point += control_point * val;
            grad_basis_array[i] = val;
            grad_t_temp += control_point * basis.Evaluate(1, t);
        }

        xVector2d vec = foot_point - point;

        // P = P0*N0+P1*N1+P2*N2+P3*N4
        // dP/dP0 = N0 and so on
        for (int i = 0; i < Order; i++) {
            control_points_grad_[index_start + i] += 2 * vec * grad_basis_array[i];
        }
        grad_t = 2 * vec.dot(grad_t_temp);

        return vec.squaredNorm();
    }

public:
    explicit BSpline(const std::vector<xVector2d> &control_points, double first_order_weight, double second_order_weight, double obstacle_Weights) {
        assert(control_points.size() >= Order);
        control_points_      = control_points;
        control_points_size_ = control_points.size();
        alpha                = first_order_weight;
        beta                 = second_order_weight;
        obstacle_Weights_    = obstacle_Weights;
        control_points_grad_.assign(control_points_size_, planning::xVector2d::Zero());
        knots_.resize(control_points_size_ + Order);
        knots_size_ = knots_.size();

        auto f_iter = knots_.begin();
        auto r_iter = knots_.rbegin();
        for (int i = 0; i < Order - 1; i++, f_iter++, r_iter++) {
            *f_iter = 0;
            *r_iter = 1;
        }
        spans_size_                = knots_size_ - 2 * Order + 1;
        Eigen::VectorXd equal_knot = Eigen::VectorXd::LinSpaced(static_cast<Eigen::Index>(spans_size_ + 1), 0, 1);
        memcpy(&knots_[Order - 1], equal_knot.data(), sizeof(double) * (spans_size_ + 1));

        basis_.reserve(control_points_size_);
        for (size_t i = 0; i < control_points_size_; i++) {
            double knots[Order + 1];
            memcpy(knots, &knots_[i], (Order + 1) * sizeof(double));
            basis_.emplace_back(knots);
        }

        order1_smooth_cost_coeff_.resize(spans_size_);
        order2_smooth_cost_coeff_.resize(spans_size_);
        weighted_basis_x_.resize(spans_size_);
        weighted_basis_y_.resize(spans_size_);

        // prepare integrate at each segment
        for (size_t span_index = 0; span_index < spans_size_; span_index++) {
            auto lb = knots_[span_index + Order - 1], ub = knots_[span_index + Order];
            Polynomial<Order - 0> poly[Order];
            Polynomial<Order - 1> d_poly[Order];
            Polynomial<Order - 2> d2_poly[Order];
            for (size_t i = 0; i < Order; i++) {
                poly[i]    = basis_[span_index + i].GetPolynomial()[Order - 1 - i];
                d_poly[i]  = Polynomial<Order - 1>(poly[i].dCoefs());
                d2_poly[i] = Polynomial<Order - 2>(poly[i].d2Coefs());
            }

            for (auto row = 0; row < Order; row++) {
                for (auto col = 0; col < Order; col++) {
                    if (col >= row) {
                        auto mul_poly1                                  = d_poly[row] * d_poly[col];
                        order1_smooth_cost_coeff_[span_index](row, col) = mul_poly1.Integral(lb, ub);
                        auto mul_poly2                                  = d2_poly[row] * d2_poly[col];
                        order2_smooth_cost_coeff_[span_index](row, col) = mul_poly2.Integral(lb, ub);
                    } else {
                        order1_smooth_cost_coeff_[span_index](row, col) =
                            order1_smooth_cost_coeff_[span_index](col, row);
                        order2_smooth_cost_coeff_[span_index](row, col) =
                            order2_smooth_cost_coeff_[span_index](col, row);
                    }
                }
            }
        }
        ResetSplineState();
    }

    xVector2d Evaluate(const std::uint32_t order, const double p) const {
        if (p < 0 or p > 1)
            return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};

        auto upper        = std::upper_bound(knots_.begin(), knots_.end(), p);
        size_t span_index = std::distance(knots_.begin(), upper) - Order;
        if (upper == knots_.end()) {
            span_index = control_points_size_ - Order;
        }

        xVector2d res{weighted_basis_x_[span_index].Evaluate(order, p),
                      weighted_basis_y_[span_index].Evaluate(order, p)};
        return res;
    }

    xVector2d EvaluateWithExtrapolation(const std::uint32_t order, const double p) const {
        if (p <= 0) {
            switch (order) {
                case 0:
                    return control_points_.front() + start_velocity_ * p;
                case 1:
                    return start_velocity_;
                default:
                    return xVector2d::Zero();
            }
        } else if (p >= 1) {
            switch (order) {
                case 0:
                    return control_points_.back() + end_velocity_ * (p - 1);
                case 1:
                    return end_velocity_;
                default:
                    return xVector2d::Zero();
            }
        }
        return Evaluate(order, p);
    }

    //获取给定点到曲线上最近点的函数
    int GetFootPoint(const xVector2d &point, const double t0, double &t) const {
        int digits                   = std::numeric_limits<double>::digits;
        int get_digits               = static_cast<int>(digits * 0.6);
        const boost::uintmax_t maxit = 20;
        boost::uintmax_t it          = maxit;
        t                            = boost::math::tools::newton_raphson_iterate(
            [this, &point](const double &tau) {
                auto P_0 = this->EvaluateWithExtrapolation(0, tau);
                auto P_1 = this->EvaluateWithExtrapolation(1, tau);
                auto P_2 = this->EvaluateWithExtrapolation(2, tau);
                auto d_P = P_0 - point;
                return boost::math::make_tuple(d_P.dot(P_1), P_1.dot(P_1) + d_P.dot(P_2));
            },
            t0, fmax(t0 - 0.1, 0.0), fmin(t0 + 0.1, 1.0), get_digits, it);
        return 0;
    }

    void CurvatureInfo(const double p, double &theta, double &kappa, double &dkappa) const {
        auto d_vec  = Evaluate(1, p);
        auto d2_vec = Evaluate(2, p);
        auto d3_vec = Evaluate(3, p);

        auto dx = d_vec.x(), dy = d_vec.y();
        auto ddx = d2_vec.x(), ddy = d2_vec.y();
        auto d3x = d3_vec.x(), d3y = d3_vec.y();

        theta           = std::atan2(dy, dx);
        auto ds_dt_pow2 = d_vec.squaredNorm();
        if (ds_dt_pow2 < kMathEps) {
            kappa  = 0;
            dkappa = 0;
            return;
        }

        auto den_p1d5 = std::pow(ds_dt_pow2, 1.5);
        auto den_p2d5 = std::pow(ds_dt_pow2, 2.5);

        auto num = dx * ddy - dy * ddx;
        kappa    = num / den_p1d5;

        auto part_a = -3 * num * (dx * ddx + dy * ddy);
        part_a /= den_p2d5;
        auto part_b = dx * d3y - dy * d3x;
        part_b /= den_p1d5;

        auto dkappa_dt = part_a + part_b;
        auto ds_dt     = sqrt(ds_dt_pow2);
        dkappa         = dkappa_dt / ds_dt;
    }

    double Theta(const double p) const {
        auto d_vec = Evaluate(1, p);
        return std::atan2(d_vec.y(), d_vec.x());
    }

    double Kappa(const double p) const {
        auto d_vec  = Evaluate(1, p);
        auto d2_vec = Evaluate(2, p);
        auto den    = d_vec.squaredNorm();
        if (den < kMathEps) {
            return 0;
        }
        auto num   = d_vec.x() * d2_vec.y() - d_vec.x() * d2_vec.x();
        auto kappa = num / std::pow(den, 1.5);
        return kappa;
    }

    const std::vector<BSplineBasis<Order>> &Basis() const { return basis_; }

    size_t ControlPointSize() const { return control_points_size_; }

    const std::vector<xVector2d> &ControlPoint() const { return control_points_; }

    std::vector<xVector2d> &MutableControlPoint() { return control_points_; }

    void ResetSplineState() {
        control_points_grad_.assign(control_points_size_, planning::xVector2d::zero());
        for (size_t span_index = 0; span_index < spans_size_; span_index++) {
            Polynomial<Order> poly_x{}, poly_y{};
            for (auto order_index = 0; order_index < Order; order_index++) {
                poly_x += basis_[span_index + order_index].GetPolynomial()[Order - 1 - order_index] *
                          control_points_[span_index + order_index].x();
                poly_y += basis_[span_index + order_index].GetPolynomial()[Order - 1 - order_index] *
                          control_points_[span_index + order_index].y();
            }
            weighted_basis_x_[span_index] = poly_x;
            weighted_basis_y_[span_index] = poly_y;
        }

        start_velocity_ = xVector2d::Zero();
        auto f_it_point = control_points_.begin();
        auto f_it_basis = basis_.begin();
        for (auto &it : start_grad_) {
            auto val = f_it_basis->Evaluate(1, 0);
            start_velocity_ += *f_it_point * val;
            it = val;
            f_it_point++;
            f_it_basis++;
        }

        end_velocity_   = xVector2d::Zero();
        auto r_it_point = control_points_.rbegin();
        auto r_it_basis = basis_.rbegin();
        for (auto &it : end_grad_) {
            auto val = r_it_basis->Evaluate(1, 1);
            end_velocity_ += *r_it_point * val;
            it = val;
            r_it_point++;
            r_it_basis++;
        }
    }

    int EqualSampleArcLength(double step, std::vector<double> &s, std::vector<double> &x,
                             std::vector<double> &y, std::vector<double> &theta,
                             std::vector<double> &kappa, std::vector<double> &dkappa) const {
        const auto d_arc_length = [this](const double t) -> double {
            auto p = Evaluate(1, t);
            return p.norm();
        };

        double arc_length = 0;
        std::vector<double> arc_length_list(spans_size_ + 1);
        arc_length_list.front() = 0;
        for (size_t span_index = 0; span_index < spans_size_; span_index++) {
            auto temp = IntegrateByGaussLegendre<5>(d_arc_length, knots_[span_index + Order - 1],
                                                    knots_[span_index + Order]);
            arc_length += temp;
            arc_length_list[span_index + 1] = arc_length;
        }

        int N                    = std::ceil(arc_length / step);
        Eigen::VectorXd s_vector = Eigen::VectorXd::LinSpaced(N, 0, arc_length);
        s.resize(N);
        x.resize(N);
        y.resize(N);
        theta.resize(N);
        kappa.resize(N);
        dkappa.resize(N);
        memcpy(&s[0], s_vector.data(), sizeof(double) * N);

        for (auto i = 0; i < N; i++) {
            auto upper = std::upper_bound(arc_length_list.begin(), arc_length_list.end(), s[i]);
            if (upper == arc_length_list.end()) {
                x[i] = control_points_.back().x();
                y[i] = control_points_.back().y();
                CurvatureInfo(1, theta[i], kappa[i], dkappa[i]);
                continue;
            }
            auto span_index = std::distance(arc_length_list.begin(), upper) - 1;
            auto s0         = arc_length_list[span_index];
            auto s1         = arc_length_list[span_index + 1];
            auto delta_s    = s[i] - s0;
            auto t0         = knots_[span_index + Order - 1];
            auto t1         = knots_[span_index + Order];
            /*
             *  / t <- try to find t in [t0, t1)               / t
             *  |    ds(t)dt = delta_s              let f(t) = | ds(t)dt - delta_s
             *  / t0                                           / t0
             */
            const auto f = [&d_arc_length, &t0, &delta_s](const double t, double &dt) -> double {
                auto fx = IntegrateByGaussLegendre<5>(d_arc_length, t0, t);
                dt      = d_arc_length(t);
                fx -= delta_s;
                return fx;
            };
            double guess = lerp(t0, t1, delta_s / (s1 - s0));
            int digits   = std::numeric_limits<double>::digits;
            auto t       = boost::math::tools::newton_raphson_iterate(arc_length_functor(f), guess, t0, t1, digits);
            auto xy      = Evaluate(0, t);
            x[i]         = xy.x();
            y[i]         = xy.y();
            CurvatureInfo(t, theta[i], kappa[i], dkappa[i]);
        }
        return 0;
    }

    const std::vector<xVector2d> &ControlPointGrad() const { return control_points_grad_; }


    double SmoothnessCost() {
        double cost = 0;
        for (size_t span_index = 0; span_index < spans_size_; span_index++) {              //遍历每个曲线段
            const auto &order1_smooth_cost_coeff = order1_smooth_cost_coeff_[span_index];  //获取当前曲线段的一阶平滑度成本系数
            const auto &order2_smooth_cost_coeff = order2_smooth_cost_coeff_[span_index];

            double x_list[Order], y_list[Order];  //控制点坐标
            for (size_t i = 0; i < Order; i++) {
                x_list[i] = control_points_[span_index + i].x();
                y_list[i] = control_points_[span_index + i].y();
            }

            // cost = p_i * p_j * N_i_4 * N_j_4
            double tempx, tempy;
            for (auto row = 0; row < Order; row++) {
                for (auto col = 0; col < Order; col++) {
                    auto order1_coeff = alpha * order1_smooth_cost_coeff(row, col);
                    auto order2_coeff = beta * order2_smooth_cost_coeff(row, col);
                    cost += order1_coeff * x_list[row] * x_list[col];
                    tempx = control_points_grad_[span_index + row].x() + order1_coeff * x_list[col];
                    control_points_grad_[span_index + row].set_x(tempx);
                    tempx = control_points_grad_[span_index + col].x() + order1_coeff * x_list[row];
                    control_points_grad_[span_index + col].set_x(tempx);

                    cost += order1_coeff * y_list[row] * y_list[col];
                    tempy = control_points_grad_[span_index + row].y() + order1_coeff * y_list[col];
                    control_points_grad_[span_index + row].set_y(tempy);
                    tempy = control_points_grad_[span_index + col].y() + order1_coeff * y_list[row];
                    control_points_grad_[span_index + col].set_y(tempy);

                    cost += order2_coeff * x_list[row] * x_list[col];
                    tempx = control_points_grad_[span_index + row].x() + order2_coeff * x_list[col];
                    control_points_grad_[span_index + row].set_x(tempx);
                    tempx = control_points_grad_[span_index + col].x() + order2_coeff * x_list[row];
                    control_points_grad_[span_index + col].set_x(tempx);

                    cost += order2_coeff * y_list[row] * y_list[col];
                    tempy = control_points_grad_[span_index + row].y() + order2_coeff * y_list[col];
                    control_points_grad_[span_index + row].set_y(tempy);
                    tempy = control_points_grad_[span_index + col].y() + order2_coeff * y_list[row];
                    control_points_grad_[span_index + col].set_y(tempy);
                }
            }
        }
        return cost;
    }

    template <typename M1, typename M2>
    double obstacle_collision(const std::vector<xVector2d> &data_points, M1 &&ts, M2 &&grad_ts, const std::vector<Eigen::MatrixXd> &A_front, const std::vector<Eigen::VectorXd> &b_front,
                              const std::vector<Eigen::MatrixXd> &A_rear, const std::vector<Eigen::VectorXd> &b_rear, const std::vector<Eigen::MatrixXd> &A_mid, const std::vector<Eigen::VectorXd> &b_mid, const std::vector<int> &front_poly_numes,
                              const std::vector<int> &rear_poly_numes, const std::vector<int> &mid_poly_numes, const std::vector<bool> &rear_optimization_flags, const std::vector<bool> &front_optimization_flags, const std::vector<bool> &mid_optimization_flags, const std::vector<xVector2d> &corners_points) {
        double cost           = 0;
        double corners_gain_R = 0.0;

        //在转弯处增加与凸包每条边的安全距离corners_gain_R(安全距离变为：corners_gain_R + safe_R)
        for (size_t i = 0; i < data_points.size(); i++) {
            corners_gain_R = 0.0;

            auto it = std::find(corners_points.begin(), corners_points.end(), data_points[i]);
            if (it != corners_points.end()) {
                corners_gain_R = 0.6;
            }

            cost += CollisionEvaluate(data_points[i], ts[i], grad_ts[i], A_front[i], b_front[i], A_rear[i], b_rear[i],  A_mid[i], b_mid[i], front_poly_numes[i], rear_poly_numes[i], mid_poly_numes[i], rear_optimization_flags[i], front_optimization_flags[i], mid_optimization_flags[i], corners_gain_R);
        }
        return cost;
    }

    double CollisionEvaluate(const xVector2d &point, double t, double &grad_t, const Eigen::MatrixXd &A_front, const Eigen::VectorXd &b_front,
                             const Eigen::MatrixXd &A_rear, const Eigen::MatrixXd &b_rear, const Eigen::MatrixXd &A_mid, const Eigen::MatrixXd &b_mid, const int &front_poly_numes, const int &rear_poly_numes, const int &mid_poly_numes,
                             const bool rear_optimization_flag, const bool front_optimization_flag, const bool mid_optimization_flag, const double corners_gain_R) {
        auto upper         = std::upper_bound(knots_.begin(), knots_.end(), t);
        size_t index_start = std::distance(knots_.begin(), upper) - Order;
        if (upper == knots_.end()) {
            index_start = control_points_size_ - Order;
        }

        xVector2d foot_point(0, 0);  //B样条拟合数据点数值
        double grad_basis_array[Order];
        xVector2d grad_t_temp(0, 0);       //B样条拟合数据点导数数值
        for (int i = 0; i < Order; i++) {  //求Order阶下 B样条拟合数据点数值 和 B样条拟合数据点导数数值
            const auto &control_point = control_points_[index_start + i];
            const auto &basis         = basis_[index_start + i];
            auto val                  = basis.Evaluate(0, t);
            foot_point += control_point * val;
            grad_basis_array[i] = val;  //求Order阶下  Order个基函数值
            grad_t_temp += control_point * basis.Evaluate(1, t);
        }

        double Ab            = 0.0;
        double fx            = 0.0;
        double grad_func_val = 0.0;
        double dists         = 0.0;
        //遍历每一个车尾安全圆心生成的凸包的每条边
        // if(rear_optimization_flag){
        for (int d = 0; d < rear_poly_numes; d++) {
            Ab    = A_rear.col(0)(d) * foot_point.x() + A_rear.col(1)(d) * foot_point.y() + b_rear(d);
            dists = fabs(Ab) / std::sqrt(pow(A_rear.col(0)(d), 2) + pow(A_rear.col(1)(d), 2));
            // if(Ab > 0.0){std::cout  << "Ab1 < 0.0   :  " << Ab << "   " << std::endl;}
            if (Ab > 0.0) {  //点在边的外侧时罚惩
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    control_points_grad_[index_start + i] +=
                        xVector2d(grad_func_val * A_rear.col(0)(d) * grad_basis_array[i], grad_func_val * A_rear.col(1)(d) * grad_basis_array[i]);
                }
            } else if (dists < corners_gain_R) {  //点在边内侧且与每条边距离小于corners_gain_R时罚惩 
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    control_points_grad_[index_start + i] +=
                        xVector2d(grad_func_val * A_rear.col(0)(d) * grad_basis_array[i], grad_func_val * A_rear.col(1)(d) * grad_basis_array[i]);
                }
            }
        }
        // }

        double gard_x  = grad_t_temp.x();
        double gard_y  = grad_t_temp.y();
        const double L_rear = 3.0;
        // if(front_optimization_flag){
        //遍历每一个车头安全圆心生成的凸包的每条边
        for (int d = 0; d < front_poly_numes; d++) {
            Ab    = A_front.col(0)(d) * (L_rear * gard_x / sqrt(gard_x * gard_x + gard_y * gard_y) + foot_point.x()) + A_front.col(1)(d) * (L_rear * gard_y / sqrt(gard_x * gard_x + gard_y * gard_y) + foot_point.y()) + b_front(d);
            dists = fabs(Ab) / std::sqrt(pow(A_front.col(0)(d), 2) + pow(A_front.col(1)(d), 2));
            // if(Ab > 0.0){std::cout  << "Ab2 < 0.0   :  " << Ab << "   " << std::endl;}
            if (Ab > 0.0) {
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    double control_points_grad_X = grad_func_val * (A_front.col(0)(d) * grad_basis_array[i] * (1.0 + L_rear * pow(gard_y, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_front.col(1)(d) * grad_basis_array[i] * L_rear * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));

                    double control_points_grad_Y = grad_func_val * (A_front.col(1)(d) * grad_basis_array[i] * (1.0 + L_rear * pow(gard_x, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_front.col(0)(d) * grad_basis_array[i] * L_rear * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));
                    control_points_grad_[index_start + i] += xVector2d(control_points_grad_X, control_points_grad_Y);
                }
            } else if (dists < corners_gain_R) {
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    double control_points_grad_X = grad_func_val * (A_front.col(0)(d) * grad_basis_array[i] * (1.0 + L_rear * pow(gard_y, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_front.col(1)(d) * grad_basis_array[i] * L_rear * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));

                    double control_points_grad_Y = grad_func_val * (A_front.col(1)(d) * grad_basis_array[i] * (1.0 + L_rear * pow(gard_x, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_front.col(0)(d) * grad_basis_array[i] * L_rear * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));
                    control_points_grad_[index_start + i] += xVector2d(control_points_grad_X, control_points_grad_Y);
                }
            }
        }
        // }


        const double L_mid = 1.5;
        //遍历每一个车中部安全圆心生成的凸包的每条边
        // if(mid_optimization_flag){
        for (int d = 0; d < mid_poly_numes; d++) {
            Ab    = A_mid.col(0)(d) * (L_mid * gard_x / sqrt(gard_x * gard_x + gard_y * gard_y) + foot_point.x()) + A_mid.col(1)(d) * (L_mid * gard_y / sqrt(gard_x * gard_x + gard_y * gard_y) + foot_point.y()) + b_mid(d);
            dists = fabs(Ab) / std::sqrt(pow(A_mid.col(0)(d), 2) + pow(A_mid.col(1)(d), 2));
            // if(Ab > 0.0){std::cout  << "Ab3 < 0.0   :  " << Ab << "   " << std::endl;}
            if (Ab > 0.0) {
            // if (dists < 1.5) {
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    double control_points_grad_X = grad_func_val * (A_mid.col(0)(d) * grad_basis_array[i] * (1.0 + L_mid * pow(gard_y, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_mid.col(1)(d) * grad_basis_array[i] * L_mid * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));

                    double control_points_grad_Y = grad_func_val * (A_mid.col(1)(d) * grad_basis_array[i] * (1.0 + L_mid * pow(gard_x, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_mid.col(0)(d) * grad_basis_array[i] * L_mid * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));
                    control_points_grad_[index_start + i] += xVector2d(control_points_grad_X, control_points_grad_Y);
                }
            } else if (dists < corners_gain_R) {
                fx += obstacle_Weights_ * pow((Ab + 15), 3);
                grad_func_val = 3 * obstacle_Weights_ * pow((Ab + 15), 2);

                for (int i = 0; i < Order; i++) {
                    double control_points_grad_X = grad_func_val * (A_mid.col(0)(d) * grad_basis_array[i] * (1.0 + L_mid * pow(gard_y, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_mid.col(1)(d) * grad_basis_array[i] * L_mid * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));

                    double control_points_grad_Y = grad_func_val * (A_mid.col(1)(d) * grad_basis_array[i] * (1.0 + L_mid * pow(gard_x, 2) / pow((gard_x * gard_x + gard_y * gard_y), 1.5)) - A_mid.col(0)(d) * grad_basis_array[i] * L_mid * gard_x * gard_y / pow((gard_x * gard_x + gard_y * gard_y), 1.5));
                    control_points_grad_[index_start + i] += xVector2d(control_points_grad_X, control_points_grad_Y);
                }
            }
        }
        // }

        return fx;
    }

    template <typename M1, typename M2>
    double FittingCost(const std::vector<xVector2d> &data_points,
                       M1 &&ts, M2 &&grad_ts) {
        double cost = 0;
        for (size_t i = 0; i < data_points.size(); i++) {
            cost += DistanceEvaluate(data_points[i], ts[i], grad_ts[i]);
        }
        return cost;
    }
};

template <int Order>
double TotalCost(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g);

template <int Order>
class BSplineSmoother {
    BSpline<Order> spline_;
    std::vector<xVector2d> data_points_;
    std::vector<BoundaryPoint> obs_comefrom_Semanticline_;
    std::vector<xVector2d> corners_points_;

public:
    BSplineSmoother(BSpline<Order> spline,
                    const std::vector<xVector2d> &data_points,
                    const std::vector<BoundaryPoint> &obs_comefrom_Semanticline,
                    const std::vector<xVector2d> &corners_points)
        : spline_(std::move(spline)), data_points_(data_points), obs_comefrom_Semanticline_(obs_comefrom_Semanticline), corners_points_(corners_points) {}

    // BSplineSmoother(BSpline<Order> spline, const std::vector<xVector2d> &data_points)
    //     : spline_(std::move(spline)), data_points_(data_points) {}

    //安全走廊参数
    std::vector<Eigen::MatrixXd> A_front;
    std::vector<Eigen::VectorXd> b_front;
    std::vector<Eigen::MatrixXd> A_rear;
    std::vector<Eigen::VectorXd> b_rear;
    std::vector<Eigen::MatrixXd> A_mid;
    std::vector<Eigen::VectorXd> b_mid;
    std::vector<int> front_poly_numes;  //车头凸包多边形条数
    std::vector<int> rear_poly_numes;
    std::vector<int> mid_poly_numes;
    std::vector<bool> rear_optimization_flags;
    std::vector<bool> front_optimization_flags;
    std::vector<bool> mid_optimization_flags;

    BSpline<Order> *SplinePtr() { return &spline_; }

    const BSpline<Order> &Spline() const { return spline_; }

    const std::vector<xVector2d> &DataPoint() const { return data_points_; }

    const std::vector<BoundaryPoint> Obs_Comefrom_Semanticline() const { return obs_comefrom_Semanticline_; }

    const std::vector<xVector2d> &Get_Corners_Points() const { return corners_points_; }

    int Optimize(std::vector<std::vector<xVector2d>> &ALL_intersection) {
        spline_.ResetSplineState();

        std::size_t data_points_size = data_points_.size();

        //储存路径点凸包参数（上限80条边）
        Eigen::MatrixXd A1(80, 2);
        Eigen::VectorXd b1(80);
        Eigen::MatrixXd A2(80, 2);
        Eigen::VectorXd b2(80);
        Eigen::MatrixXd A3(80, 2);
        Eigen::VectorXd b3(80);
        rear_optimization_flags.clear();
        front_optimization_flags.clear();
        mid_optimization_flags.clear();

        //前圆心参数
        static const double center_distance = 3.0;
        static const double half_center_distance = 1.5;
        static const double safe_radius     = 1.5;  //安全圆半径
        double front_center_x = 0.0;
        double front_center_y = 0.0;
        double mid_center_x = 0.0;
        double mid_center_y = 0.0;
        double local_s = 0.0;
        double dx = 0.0, dy = 0.0;
        Eigen::VectorXd heading(data_points_size);
std::ofstream obstest("/log/hpa_routes/obstest_rear.csv");
if (!obstest.is_open()) {
    std::cerr << "Error opening obstest: " << "obstest_rear.csv" << std::endl;
    // return;
}
        std::vector<std::vector<xVector2d>> ALL_intersection_front;
        std::vector<std::vector<xVector2d>> ALL_intersection_rear;
        std::vector<std::vector<xVector2d>> ALL_intersection_mid;
        for (std::size_t i = 0; i < data_points_size; i++) {
            //后轴中心
            xVector2d rear_center(data_points_[i].x(), data_points_[i].y());

            //航向方向
            xVector2d heading_vector;
            if (i == 0) {
                heading_vector = normalizeVector(xVector2d(data_points_[i + 1].x() - data_points_[i].x(),
                                                           data_points_[i + 1].y() - data_points_[i].y()));
            } else if (i == (data_points_size - 1)) {
                heading_vector = normalizeVector(xVector2d(data_points_[i].x() - data_points_[i - 1].x(),
                                                           data_points_[i].y() - data_points_[i - 1].y()));
            } else {
                heading_vector = normalizeVector(xVector2d(data_points_[i + 1].x() - data_points_[i - 1].x(),
                                                           data_points_[i + 1].y() - data_points_[i - 1].y()));
            }
            heading(i) = atan2(heading_vector.y(), heading_vector.x());

            //前轴碰撞圆圆心
            front_center_x = heading_vector.x() * center_distance + rear_center.x();
            front_center_y = heading_vector.y() * center_distance + rear_center.y();
            xVector2d front_center(front_center_x, front_center_y);

            //中間碰撞圆圆心
            mid_center_x = heading_vector.x() * half_center_distance + rear_center.x();
            mid_center_y = heading_vector.y() * half_center_distance + rear_center.y();
            xVector2d mid_center(mid_center_x, mid_center_y);

            xVector2d next_rear_center;
            if(i < data_points_size-1){
                next_rear_center = xVector2d(data_points_[i+1].x(), data_points_[i+1].y());
            }else{
                next_rear_center = xVector2d(data_points_[i].x(), data_points_[i].y());
            }
            local_s += std::sqrt(dx * dx + dy * dy);
            dx = next_rear_center.x() - rear_center.x();
            dy = next_rear_center.y() - rear_center.y();
            
std::vector<xVector2d> obstest_rear;
std::vector<xVector2d> obstest_front;
std::vector<xVector2d> obstest_mid;

            //构建前后圆心的安全走廊    std::vector<bool> optimization_flags
            get_safe_area(rear_optimization_flags, obstest_rear, obs_comefrom_Semanticline_, rear_center, local_s, safe_radius, A1, b1, rear_poly_numes, ALL_intersection_rear);
            get_safe_area(front_optimization_flags, obstest_front, obs_comefrom_Semanticline_, front_center, local_s + center_distance, safe_radius, A2, b2, front_poly_numes, ALL_intersection_front);
            get_safe_area(mid_optimization_flags, obstest_mid, obs_comefrom_Semanticline_, mid_center, local_s + half_center_distance, safe_radius, A3, b3, mid_poly_numes, ALL_intersection_mid);
            A_front.push_back(A2);
            b_front.push_back(b2);
            A_rear.push_back(A1);
            b_rear.push_back(b1);
            A_mid.push_back(A3);
            b_mid.push_back(b3);

// 写入数据行
for (size_t i = 0; i < obstest_rear.size(); ++i) {
        obstest << obstest_rear[i].x();
        obstest << ",";
        obstest << obstest_rear[i].y();
        obstest << ",";
}
obstest << "\n";
obstest << rear_center.x();
obstest << ",";
obstest << rear_center.y();
obstest << ",";
obstest << local_s;
obstest << "\n";
        }
obstest.close(); 

        ALL_intersection = ALL_intersection_rear;

        auto start_time = std::chrono::steady_clock::now();
        std::vector<double> knots_fitting;
        knots_fitting.resize(data_points_size);

        auto match_point_size = std::max(100UL, data_points_size);  //离散t的点数
        Eigen::VectorXd ts    = Eigen::VectorXd::LinSpaced(         //在0到1上分布t
            static_cast<Eigen::Index>(match_point_size), 0, 1);

        std::vector<xVector2d> point_to_match(match_point_size);  //每个t对应的基
        for (auto i = 0; i < ts.size(); i++) {
            point_to_match[i] = spline_.Evaluate(0, ts[i]);
        }

        auto lb = point_to_match.begin();
        for (size_t i = 0; i < data_points_size; i++) {  //将每一个行车点对应 ts(i) knots_fitting[i]
            const auto &point = data_points_[i];
            auto min_iter     = std::min_element(lb, point_to_match.end(),  //找到 基 中距离 行车点 最近基的索引
                                             [&point](const xVector2d &a, xVector2d &b) {
                                                 return (a - point).squaredNorm() < (b - point).squaredNorm();
                                             });
            double distance   = (*min_iter - point).squaredNorm();  //索引距离
            (void)distance;
            auto min_index   = std::distance(point_to_match.begin(), min_iter);
            knots_fitting[i] = ts[min_index];
            lb               = min_iter;
        }

        // update knots to optimized knots
        for (size_t i = 0; i < data_points_size; i++) {
            spline_.GetFootPoint(data_points_[i], knots_fitting[i], knots_fitting[i]);
        }

        double finalCost;
        auto control_points_size = spline_.ControlPointSize();
        auto knots_start_index   = static_cast<Eigen::Index>(2 * (control_points_size - 2));  //有效节点起始索引
        Eigen::VectorXd x(knots_start_index + data_points_size);

        /* Set the initial guess */
        for (size_t i = 1, cnt = 0; i < control_points_size - 1; i++) {
            x(cnt++) = spline_.ControlPoint()[i].x();
            x(cnt++) = spline_.ControlPoint()[i].y();
        }
        for (size_t i = 0; i < data_points_size; i++) {
            x(knots_start_index + i) = knots_fitting[i];
        }

        /* Set the minimization parameters */
        lbfgs::lbfgs_parameter_t params;
        params.mem_size  = 10;
        params.g_epsilon = 1.0e-8;
        params.past      = 3;
        params.delta     = 1.0e-8;

        auto len = this->data_points_.size();
        (void)len;

        /* Start minimization */
        int ret = lbfgs::lbfgs_optimize(x,
                                        finalCost,
                                        TotalCost<Order>,
                                        nullptr,
                                        nullptr,
                                        this,
                                        params);

        /* Report the result. */
        std::cout << std::setprecision(4)
                  << "================================" << std::endl
                  << "L-BFGS Optimization Returned: " << ret << std::endl
                  << "Minimized Cost: " << finalCost << std::endl;
        // std::cout << "Optimal Variables: \n" << x.transpose() << std::endl;

        auto current_time = std::chrono::steady_clock::now();
        double period =
            std::chrono::duration<double, std::milli>(current_time - start_time).count();
        std::cout << "lbfgs_optimize computer time:" << period << " ms" << std::endl;
        return ret;
    }
};

template <int Order>
double TotalCost(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    auto *smoother_ptr = reinterpret_cast<BSplineSmoother<Order> *>(instance);
    auto *spline_ptr   = smoother_ptr->SplinePtr();

    auto knots_size = static_cast<Eigen::Index>(smoother_ptr->DataPoint().size());
    for (size_t i = 1, cnt = 0; i < spline_ptr->ControlPointSize() - 1; i++, cnt += 2) {
        spline_ptr->MutableControlPoint()[i] = {x[cnt + 0], x[cnt + 1]};
    }
    spline_ptr->ResetSplineState();
    g.setZero();

    // //将优化前的所有数据记录到CSV
    // #ifdef BUILD_IN_TDA4
    // write_data(smoother_ptr->DataPoint(), smoother_ptr->A_front, smoother_ptr->b_front, smoother_ptr->A_rear,
    //            smoother_ptr->b_rear, smoother_ptr->front_poly_numes, smoother_ptr->rear_poly_numes, smoother_ptr->Get_Corners_Points());
    // #endif 

    double cost = 0;

    cost += spline_ptr->SmoothnessCost();
    cost += spline_ptr->FittingCost(smoother_ptr->DataPoint(),
                                    x.bottomRows(knots_size),
                                    g.bottomRows(knots_size));
    cost += spline_ptr->obstacle_collision(smoother_ptr->DataPoint(),
                                           x.bottomRows(knots_size),
                                           g.bottomRows(knots_size),
                                           smoother_ptr->A_front,
                                           smoother_ptr->b_front,
                                           smoother_ptr->A_rear,
                                           smoother_ptr->b_rear,
                                           smoother_ptr->A_mid,
                                           smoother_ptr->b_mid,
                                           smoother_ptr->front_poly_numes,
                                           smoother_ptr->rear_poly_numes,
                                           smoother_ptr->mid_poly_numes,
                                           smoother_ptr->rear_optimization_flags,
                                           smoother_ptr->front_optimization_flags,
                                           smoother_ptr->mid_optimization_flags,
                                           smoother_ptr->Get_Corners_Points());

// //读取PC优化前数据测试tda4优化器
// std::vector<xVector2d> read_DataPoint;
//     std::vector<Eigen::MatrixXd> read_A_front;
//     std::vector<Eigen::MatrixXd> read_A_rear;
//     std::vector<Eigen::VectorXd> read_b_front;
//     std::vector<Eigen::VectorXd> read_b_rear;
//     std::vector<int> read_front_poly_numes;
//     std::vector<int> read_rear_poly_numes;
//     std::vector<xVector2d> read_corners_points;
//     read_csv_data(read_DataPoint, read_A_front, read_A_rear, read_b_front, read_b_rear, read_front_poly_numes, read_rear_poly_numes, read_corners_points);

//     cost += spline_ptr->SmoothnessCost();
//     // cost += spline_ptr->FittingCost(read_DataPoint,
//     //                                 x.bottomRows(knots_size),
//     //                                 g.bottomRows(knots_size)); 
//     cost += spline_ptr->obstacle_collision(smoother_ptr->DataPoint(),
//                                     x.bottomRows(knots_size),
//                                     g.bottomRows(knots_size),
//                                     read_A_front,
//                                     read_b_front,
//                                     read_A_rear,
//                                     read_b_rear,
//                                     read_front_poly_numes,
//                                     read_rear_poly_numes,
//                                     read_corners_points);  

    for (size_t i = 1, cnt = 0; i < spline_ptr->ControlPointSize() - 1; i++) {
        g[cnt++] += spline_ptr->ControlPointGrad()[i].x();
        g[cnt++] += spline_ptr->ControlPointGrad()[i].y();
    }
    return cost;
}
}  // namespace planning
