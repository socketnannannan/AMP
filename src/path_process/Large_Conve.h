#pragma once
#include <quickhull.h>
#include "matplotlibcpp.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>

// #ifdef M_PI
    # define M_PI		3.14159265358979323846	/* pi */
// #endif

// # define PICTURE

using namespace planning;

// 计算多边形面积
double polygonArea(const std::vector<xVector2d>& vertices) {
    int n = vertices.size();
    double area = 0.0;

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += (vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y());
    }

    area = 0.5 * std::abs(area);
    return area;
}

// 计算超平面交点
inline bool line_intersect(const Simple_Triangle& A, const Simple_Triangle& B, xVector2d& pi) {
    double a1 = -A.v.y();
    double b1 = A.v.x();
    double c1 = a1 * A.hull_point.x() + b1 * A.hull_point.y();

    double a2 = -B.v.y();
    double b2 = B.v.x();
    double c2 = a2 * B.hull_point.x() + b2 * B.hull_point.y();

    double x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
    double y = (c1 * a2 - c2 * a1) / (a2 * b1 - a1 * b2);

    if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) {
        return false;
    } else {
        pi = xVector2d(x, y);
        return true;
    }
}

// 计算两点之间的极角（角度）
double calculateAngle(const xVector2d& center, const xVector2d& p) {
    double angle = std::atan2(p.y() - center.y(), p.x() - center.x());
    // if(angle < 0){
    //     angle = angle + 2 * M_PI;
    // }
    return angle;
}

// 判断点 p1 是否比点 p2 极角更小（逆时针）
bool comparePoints(const xVector2d& center, const xVector2d& p1, const xVector2d& p2) {
    double angle1 = calculateAngle(center, p1);
    double angle2 = calculateAngle(center, p2);

    if (angle1 == angle2) {
        // 如果两点有相同的极角，按照它们到圆心的距离排序
        double dist1 = std::hypot(p1.x() - center.x(), p1.y() - center.y());
        double dist2 = std::hypot(p2.x() - center.x(), p2.y() - center.y());
        return dist1 < dist2;
    }
    return angle1 < angle2;
}

//对圆上的点进行逆时针排序
void sortPointsClockwise(std::vector<xVector2d>& points, const xVector2d& center) {
    std::sort(points.begin(), points.end(), [&](const xVector2d& p1, const xVector2d& p2) {
        return comparePoints(center, p1, p2);
    });
}

std::vector<xVector2d> Large_Conve(const xVector2d& center, std::vector<xVector2d>& obs, const double& safe_R) {
    //辅助点
    static const double R_help = 3.5;
    static const int nume_help = 48;
    for (int i = 0; i < nume_help; ++i) {
        xVector2d obs_help;
        double angle = 2 * M_PI * i / nume_help;
        obs_help.set_x(R_help * cos(angle) + center.x());
        obs_help.set_y(R_help * sin(angle) + center.y());
        obs.push_back(obs_help);
    }
#ifdef PICTURE
//障礙物
    namespace plt = matplotlibcpp;
    std::vector<double> x_s, y_s;
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < obs.size(); j++) {
        x_s.emplace_back(obs[j].x());
        y_s.emplace_back(obs[j].y());
    }
    plt::scatter(x_s, y_s, 10);
#endif 

    //障碍物点外翻，生成外翻的障碍物点
    static const double R = 20.0;  //障碍物点外翻半径
    std::vector<xVector2d> obs_out;
    for (xVector2d& point1 : obs) {
        point1.set_x(point1.x() - center.x());
        point1.set_y(point1.y() - center.y());
        xVector2d point_out;
        double point_in_value = std::sqrt(point1.x() * point1.x() + point1.y() * point1.y());
        point_out.set_x(point1.x() * (2 * R - point_in_value) / point_in_value + center.x());
        point_out.set_y(point1.y() * (2 * R - point_in_value) / point_in_value + center.y());
        obs_out.push_back(point_out);
    }

    //生成外翻障碍物点的凸包外壳
    std::vector<xVector2d> convexHull_out;
    convex_hull(obs_out);
    convexHull_out = obs_out;
    sortPointsClockwise(convexHull_out, center);

    //外翻的障碍物点凸包外壳反向内翻 生成星型凸包
    std::vector<xVector2d> star_convexhull;
    for (xVector2d& point2 : convexHull_out) {
        xVector2d point_in;
        point2.set_x(point2.x() - center.x());
        point2.set_y(point2.y() - center.y());
        double point_out_value = std::sqrt(point2.x() * point2.x() + point2.y() * point2.y());
        point_in.set_x(point2.x() * (2 * R - point_out_value) / point_out_value + center.x());
        point_in.set_y(point2.y() * (2 * R - point_out_value) / point_out_value + center.y());
        star_convexhull.push_back(point_in);
    }
    sortPointsClockwise(star_convexhull, center);

    //分解星型凸包得到外壳和内凹点
    std::vector<xVector2d> star_convexhulls = star_convexhull;
    std::vector<xVector2d> convexHull_in;  //星型凸包外壳
    std::vector<xVector2d> concave_point;  //外壳凸包内凹点
    convex_hull(star_convexhull);
    convexHull_in = star_convexhull;
    for (auto it = star_convexhulls.begin(); it != star_convexhulls.end(); ++it) {
        const xVector2d& currentPoint = *it;
        auto itt                      = std::find(convexHull_in.begin(), convexHull_in.end(), currentPoint);
        if (itt == convexHull_in.end()) {
            concave_point.push_back(currentPoint);
        }
    }
    sortPointsClockwise(convexHull_in, center);

    std::vector<Simple_Triangle> Hull;  //单纯三角集
    //为单纯三角存入极点、法向、极角
    for (auto it = convexHull_in.begin(); it != convexHull_in.end(); ++it) {
        const xVector2d& currentPoint = *it;
        auto nextIt                   = std::next(it);
        if (nextIt == convexHull_in.end()) {
            nextIt = convexHull_in.begin();
        }
        const xVector2d& nextPoint = *nextIt;

        Simple_Triangle simple_triangle;
        simple_triangle.hull_point = currentPoint;
        simple_triangle.v          = xVector2d(nextPoint.x() - currentPoint.x(), nextPoint.y() - currentPoint.y());
        //保证法向量指向凸包内部
        simple_triangle.n = normalizeVector(xVector2d(simple_triangle.v.y(), -1 * simple_triangle.v.x()));
        if ((center.x() - currentPoint.x()) * simple_triangle.n.x() + (center.y() - currentPoint.y()) * simple_triangle.n.y() < 0.001) {
            simple_triangle.n = normalizeVector(xVector2d(-1 * simple_triangle.v.y(), simple_triangle.v.x()));
        }
        simple_triangle.beta = calculateAngle(center, currentPoint);
        Hull.push_back(simple_triangle);
    }

    //为每个单纯三角存入相应内部的外壳凸包内凹点
    for (const xVector2d& point3 : concave_point) {
        double point_beta = calculateAngle(center, point3);
        for (auto it = Hull.begin(); it != Hull.end(); ++it) {
            if (it->beta >= point_beta) {
                if (it != Hull.begin()) {
                    auto prevIt1 = std::prev(it);
                    prevIt1->inpoint.push_back(point3);
                } else {
                    auto prevIt2 = std::prev(Hull.end());
                    prevIt2->inpoint.push_back(point3);
                }
                break;
            } else if (point_beta > std::prev(Hull.end())->beta) {
                auto prevIt3 = std::prev(Hull.end());
                prevIt3->inpoint.push_back(point3);
                break;
            }
        }
    }

    //收缩极边到单纯三角内部最远点，构建最小凸包
    std::vector<xVector2d> Hull_Point;  //收缩后凸包每条边的锚定点
    for (auto it = Hull.begin(); it != Hull.end(); ++it) {
        auto nextIt = std::next(it);
        if (nextIt == Hull.end()) {
            nextIt = Hull.begin();
        }
        double mindist    = 0.0;
        xVector2d dis_max = it->hull_point;
        for (const xVector2d& point4 : it->inpoint) {
            double distance = dist_pointtoline(it->hull_point, nextIt->hull_point, point4);
            if (mindist < distance) {
                mindist = distance;
                dis_max = point4;
            }
        }
        Hull_Point.push_back(dis_max);
    }

    //更新单纯三角中的每条边的锚定点
    auto it  = Hull.begin();
    auto itt = Hull_Point.begin();
    for (; it != Hull.end() && itt != Hull_Point.end(); ++it, ++itt) {
        it->hull_point = *itt;
    }
    if(Hull.size() != Hull_Point.size()){
        std::cout << "something false in update Hull_Point" << std::endl;
    }
#ifdef PICTURE
// 星型凸包外壳
std::vector<xVector2d> gggg;
gggg = convexHull_in;
// 收缩後
std::vector<xVector2d> yyyy;
    for (auto it = Hull.begin(); it != Hull.end(); ++it) {
        yyyy.push_back(it->hull_point);
    }
//外壳凸包内凹点
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < concave_point.size(); j++) {
        x_s.emplace_back(concave_point[j].x());
        y_s.emplace_back(concave_point[j].y());
    }
    plt::scatter(x_s, y_s, 15);

std::vector<Simple_Triangle> Hull11 = Hull;
//将所有超平面向内收缩safe_R，获得由满足点到满足车辆长方体的安全走廊
for (auto it = Hull11.begin(); it != Hull11.end(); ++it) {
    it->hull_point = it->hull_point + it->n * 0.0;
}
//计算所有超平面交点
std::vector<xVector2d> intersection_pointall11;  //超平面交点
for (auto i = Hull11.begin(); i != Hull11.end(); i++) {
    for (auto j = i + 1; j != Hull11.end(); j++) {
        xVector2d pi;
        if (line_intersect(*i, *j, pi)) {  //判断相交并计算交点
            intersection_pointall11.push_back(pi);
        }
    }
}

//剔去在所有超平面外部的交点 
const double epsilon_threshold11 = 0.01;
std::vector<xVector2d> intersection_pointin11 = intersection_pointall11;
for (const xVector2d& point5 : intersection_pointall11) {
    for (const Simple_Triangle& st : Hull11) {
        xVector2d st_n_normalize = normalizeVector(st.n); // 归一化法线
        xVector2d point5_normalize = normalizeVector(point5 - st.hull_point); // 归一化点
        if (st_n_normalize.dot(point5_normalize)/(st_n_normalize.norm()*point5_normalize.norm()) < -epsilon_threshold11) {
        // if ((st.n.x() * (point5.x() - st.hull_point.x()) + st.n.y() * (point5.y() - st.hull_point.y())) < -epsilon_threshold) { //-1.0 * std::numeric_limits<double>::epsilon()
            intersection_pointin11.erase(std::remove(intersection_pointin11.begin(), intersection_pointin11.end(), point5), intersection_pointin11.end());
            break;
        }
    }
}
//因为为将车头尾视为点，上文将超平面向内收缩safe_R导致行车点可能在多边形外时，使用标准点进行排序
xVector2d average_point11(0.0, 0.0);  //交点标准中心
for (const xVector2d& point5 : intersection_pointin11) {
    average_point11.set_x(average_point11.x() + point5.x());
    average_point11.set_y(average_point11.y() + point5.y());
}
average_point11.set_x(average_point11.x() / intersection_pointin11.size());
average_point11.set_y(average_point11.y() / intersection_pointin11.size());
sortPointsClockwise(intersection_pointin11, average_point11);  //逆时针排序
//凸包外壳
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < intersection_pointin11.size(); j++) {
        x_s.emplace_back(intersection_pointin11[j].x());
        y_s.emplace_back(intersection_pointin11[j].y());
    }
    x_s.emplace_back(intersection_pointin11[0].x());
    y_s.emplace_back(intersection_pointin11[0].y());
    plt::plot(x_s, y_s, "black");
#endif

    //将所有超平面向内收缩safe_R，获得由满足点到满足车辆长方体的安全走廊
    for (auto it = Hull.begin(); it != Hull.end(); ++it) {
        it->hull_point = it->hull_point + it->n * safe_R;
    }

    //计算所有超平面交点
    std::vector<xVector2d> intersection_pointall;  //超平面交点
    for (auto i = Hull.begin(); i != Hull.end(); i++) {
        for (auto j = i + 1; j != Hull.end(); j++) {
            xVector2d pi;
            if (line_intersect(*i, *j, pi)) {  //判断相交并计算交点
                intersection_pointall.push_back(pi);
            }
        }
    }

    //剔去在所有超平面外部的交点 
    const double epsilon_threshold = 0.01;
    std::vector<xVector2d> intersection_pointin = intersection_pointall;
    for (const xVector2d& point5 : intersection_pointall) {
        for (const Simple_Triangle& st : Hull) {
            xVector2d st_n_normalize = normalizeVector(st.n); // 归一化法线
            xVector2d point5_normalize = normalizeVector(point5 - st.hull_point); // 归一化点
            if (st_n_normalize.dot(point5_normalize)/(st_n_normalize.norm()*point5_normalize.norm()) < -epsilon_threshold) {
            // if ((st.n.x() * (point5.x() - st.hull_point.x()) + st.n.y() * (point5.y() - st.hull_point.y())) < -epsilon_threshold) { //-1.0 * std::numeric_limits<double>::epsilon()
                intersection_pointin.erase(std::remove(intersection_pointin.begin(), intersection_pointin.end(), point5), intersection_pointin.end());
                break;
            }
        }
    }

    //因为为将车头尾视为点，上文将超平面向内收缩safe_R导致行车点可能在多边形外时，使用标准点进行排序
    xVector2d average_point(0.0, 0.0);  //交点标准中心
    for (const xVector2d& point5 : intersection_pointin) {
        average_point.set_x(average_point.x() + point5.x());
        average_point.set_y(average_point.y() + point5.y());
    }
    average_point.set_x(average_point.x() / intersection_pointin.size());
    average_point.set_y(average_point.y() / intersection_pointin.size());
    sortPointsClockwise(intersection_pointin, average_point);  //逆时针排序

    // 假如凸包过小不合理则置空
    if((polygonArea(intersection_pointin) <= M_PI*1.5*1.5/6) || (intersection_pointin.empty())){
        intersection_pointin.clear();

        intersection_pointin.emplace_back(center + xVector2d(0.1, 0));
        intersection_pointin.emplace_back(center + xVector2d(0, -0.1));
        intersection_pointin.emplace_back(center + xVector2d(-0.1, 0));
        intersection_pointin.emplace_back(center + xVector2d(0, 0.1));

    }


#ifdef PICTURE
//-------------------------------------------


    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < intersection_pointin.size(); j++) {
        x_s.emplace_back(intersection_pointin[j].x());
        y_s.emplace_back(intersection_pointin[j].y());
    }
    x_s.emplace_back(intersection_pointin[0].x());
    y_s.emplace_back(intersection_pointin[0].y());
    plt::plot(x_s, y_s, "r");


    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < yyyy.size(); j++) {
        x_s.emplace_back(yyyy[j].x());
        y_s.emplace_back(yyyy[j].y());
    }
    x_s.emplace_back(yyyy[0].x());
    y_s.emplace_back(yyyy[0].y());
    plt::plot(x_s, y_s, "y");

    x_s.clear();
    y_s.clear();
    std::vector<xVector2d> dddd;
    for (int i = 0; i < 24; ++i) {
        xVector2d obs_helps;
        double angle = 2 * M_PI * i / 24;
        obs_helps.set_x(1.5 * cos(angle) + center.x());
        obs_helps.set_y(1.5 * sin(angle) + center.y());
        dddd.push_back(obs_helps);
    }
    for (std::size_t j = 0; j < dddd.size(); j++) {
        x_s.emplace_back(dddd[j].x());
        y_s.emplace_back(dddd[j].y());
    }
    x_s.emplace_back(dddd[0].x());
    y_s.emplace_back(dddd[0].y());
    plt::plot(x_s, y_s, "y--");

//星型凸包外壳
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < gggg.size(); j++) {
        x_s.emplace_back(gggg[j].x());
        y_s.emplace_back(gggg[j].y());
    }
    x_s.emplace_back(gggg[0].x());
    y_s.emplace_back(gggg[0].y());
    plt::plot(x_s, y_s, "g");


    x_s.clear();
    y_s.clear();
    x_s.emplace_back(center.x());
    y_s.emplace_back(center.y());
    
    plt::scatter(x_s, y_s, 20);  


        plt::axis("equal");  
        plt::show();
//-------------------------------------------
#endif

    return intersection_pointin;
}
