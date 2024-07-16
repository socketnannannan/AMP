#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include "common.h"

using namespace planning;

//向量单位化
xVector2d normalizeVector(const xVector2d& vector) {
    double length = sqrt(vector.x() * vector.x() + vector.y() * vector.y());
    if (length != 0.0) {
        return xVector2d(vector.x() / length, vector.y() / length);
    } else {
        // 处理零向量的情况，例如返回默认向量或抛出异常
        // 这里简单返回一个默认向量
        return xVector2d(0.0, 0.0);
    }
}

//点到线段距离
double dist_pointtoline(const xVector2d& A, const xVector2d& B, const xVector2d& P) {
    double dists;
    double A_coeff, B_coeff, C_coeff;

    // 计算直线的系数 A, B, C   Ax+By+C=0
    if (A.x() != B.x()) {
        A_coeff = B.y() - A.y();
        B_coeff = A.x() - B.x();
        C_coeff = A.y() * B.x() - A.x() * B.y();
    } else {
        // 如果直线垂直于 x 轴
        A_coeff = 1;
        B_coeff = 0;
        C_coeff = -A.x();
    }

    // 计算点到直线的距离
    if (A_coeff != 0 || B_coeff != 0) {
        dists = fabs(A_coeff * P.x() + B_coeff * P.y() + C_coeff) / std::sqrt(A_coeff * A_coeff + B_coeff * B_coeff);
    } else {
        // 如果直线是一个点
        dists = std::sqrt((P.x() - A.x()) * (P.x() - A.x()) + (P.y() - A.y()) * (P.y() - A.y()));
    }

    return dists;
}

int orientation(xVector2d a, xVector2d b, xVector2d c) {
    double v = a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y());
    if (v < 0) return -1;  // clockwise
    if (v > 0) return +1;  // counter-clockwise
    return 0;
}

bool cw(xVector2d a, xVector2d b, xVector2d c, bool include_collinear) {
    int o = orientation(a, b, c);
    return o < 0 || (include_collinear && o == 0);
}
bool ccw(xVector2d a, xVector2d b, xVector2d c, bool include_collinear) {
    int o = orientation(a, b, c);
    return o > 0 || (include_collinear && o == 0);
}

//最大凸包外壳
void convex_hull(std::vector<xVector2d>& a, bool include_collinear = false) {
    if (a.size() == 1)
        return;

    sort(a.begin(), a.end(), [](xVector2d a, xVector2d b) {
        return std::make_pair(a.x(), a.y()) < std::make_pair(b.x(), b.y());
    });
    xVector2d p1 = a[0], p2 = a.back();
    std::vector<xVector2d> up, down;
    up.push_back(p1);
    down.push_back(p1);
    for (size_t i = 1; i < a.size(); i++) {
        if (i == a.size() - 1 || cw(p1, a[i], p2, include_collinear)) {
            while (up.size() >= 2 && !cw(up[up.size() - 2], up[up.size() - 1], a[i], include_collinear))
                up.pop_back();
            up.push_back(a[i]);
        }
        if (i == a.size() - 1 || ccw(p1, a[i], p2, include_collinear)) {
            while (down.size() >= 2 && !ccw(down[down.size() - 2], down[down.size() - 1], a[i], include_collinear))
                down.pop_back();
            down.push_back(a[i]);
        }
    }

    if (include_collinear && up.size() == a.size()) {
        reverse(a.begin(), a.end());
        return;
    }
    a.clear();
    for (size_t i = 0; i < up.size(); i++)
        a.push_back(up[i]);
    for (size_t i = down.size() - 2; i > 0; i--)
        a.push_back(down[i]);
}
