#include "path_process.h"
#include <iostream>
#include <fstream>
#include <memory>
#include "bspline.h"
#include "matplotlibcpp.h"
#include "path_process_api.h"
#include "sampler.h"
using namespace planning;

xVector2d normalizeVectorxVector2d(const xVector2d& vector) {
    double length = sqrt(vector.x() * vector.x() + vector.y() * vector.y());
    if (length != 0.0) {
        return xVector2d(vector.x() / length, vector.y() / length);
    } else {
        // 处理零向量的情况，例如返回默认向量或抛出异常
        // 这里简单返回一个默认向量
        return xVector2d(0.0, 0.0);
    }
}

void read_data(std::vector<ExPathPoint> &data_point, std::vector<BoundaryPoint> &obs_comefrom_Semanticline_left, std::vector<BoundaryPoint> &obs_comefrom_Semanticline_right, std::vector<xVector2d>& Trajectory_from_tda4){
  
    // //从bin读取录制的原轨迹 需要試驗預處理請用bin文件回灌
    // MapIO map_io;   //相对路径  offline_debug/data_front_optimization/11.2/hpa_routes/map2_2024-1-11-14-0-38.bin
    // map_io.LoadMap("offline_debug/data_front_optimization/11.2/HA1_707_2023-12-27-14-55-19/map1_2023-12-27-14-55-0.bin");
    // std::vector<MapPosRecord> map_path;
    // auto res = map_io.GetMapPath(map_path);
    // if (!res){
    //     std::cout << "读取bin失败" << std::endl;
    // }

    // // std::vector<ExPathPoint> pre_path_out;
    // res = ExPathPreprocess(map_path, data_point);
    // // for (const auto &point : pre_path_out) {
    // //     data_point.emplace_back(point.x, point.y);
    // // }  
  
    //read 使用預处理后轨迹進行回灌（需要試驗預處理請用以上bin文件回灌）
    // 疯狂直线s形形式＋内切的包
    std::ifstream TempIns1("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes2024.1.15.02/2024-01-15-13-48-07plan1.csv");    //s行驶的包
    // std::ifstream TempIns1("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes20141.12.05/2024-01-12-16-38-35plan1.csv");  //很长的包
    // std::ifstream TempIns1("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes_cjh/2024-01-16-16-03-30plan1.csv");      //很多贴墙的包
    std::string line1;
    // Skip the first line
    getline(TempIns1, line1);
    while (getline(TempIns1, line1)) {
        std::istringstream s(line1);       
        std::vector<std::string> fields1;  
        std::string field1;
        while (getline(s, field1, ','))  
        {
            fields1.push_back(field1); 
        }
        double right_xx = std::stod(fields1[0]);
        double right_yy = std::stod(fields1[1]);
        data_point.emplace_back(ExPathPoint(right_xx, right_yy, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    //read obs
    std::ifstream TempIns("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes2024.1.15.02/map3_2024-1-15-13-48-3-planning.csv");
    // std::ifstream TempIns("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes20141.12.05/map2_2024-1-12-16-38-29-planning.csv");
    // std::ifstream TempIns("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes_cjh/map2_2024-1-16-16-3-26-planning.csv");
    std::string line;
    // Skip the first line
    getline(TempIns, line);
    while (getline(TempIns, line)) {
        std::istringstream s(line);  
        std::vector<std::string> fields;  
        std::string field;
        while (getline(s, field, ','))  
        {
            fields.push_back(field);  
        }
        double right_x = std::stod(fields[0]);
        double right_y = std::stod(fields[1]);
        double right_s = std::stod(fields[2]);
        obs_comefrom_Semanticline_right.emplace_back(right_x, right_y, right_s);
        double left_x = std::stod(fields[3]);
        double left_y = std::stod(fields[4]);
        double left_s = std::stod(fields[5]);
        obs_comefrom_Semanticline_left.emplace_back(left_x, left_y, left_s);
    }
    
    //read tda4 optimization Trajectory 
    std::ifstream TempIns2("/home/olavnan/map_io(new)/map_io/offline_debug/data_front_optimization/11.2/hpa_routes_cjh/map2_2024-1-16-16-3-26.csv");
    std::string line2;
    // Skip the first line
    getline(TempIns2, line2);
    while (getline(TempIns2, line2)) {
        std::istringstream s(line2);  
        std::vector<std::string> fields2;  
        std::string field2;
        while (getline(s, field2, ','))  
        {
            fields2.push_back(field2);  
        }
        double xxx = std::stod(fields2[0]);
        double yyy = std::stod(fields2[1]);
        Trajectory_from_tda4.emplace_back(xxx, yyy);
    }

}

void Get_Offline_Data(std::vector<ExPathPoint> &exPathPoint, std::vector<BoundaryPoint> &left_boundary, std::vector<BoundaryPoint> &right_boundary, std::vector<xVector2d>& Trajectory_from_tda4){
    exPathPoint.clear();
    left_boundary.clear();
    right_boundary.clear();
    // std::vector<xVector2d> data_point;
    // std::vector<BoundaryPoint> obs_comefrom_Semanticline_left;
    // std::vector<BoundaryPoint> obs_comefrom_Semanticline_right;
    read_data(exPathPoint, left_boundary, right_boundary, Trajectory_from_tda4);

    // for(const xVector2d& it : data_point){
    //     exPathPoint.push_back(ExPathPoint(it.x(), it.y(), 0, 0, 0, 0, 0));
    // }
    // for(const xVector2d& it : obs_comefrom_Semanticline_left){
    //     BoundaryPoint point;
    //     point.x = it.x();
    //     point.y = it.y();
    //     point.s = it.s();
    //     left_boundary.push_back(point);
    // }

    // for(const xVector2d& it : obs_comefrom_Semanticline_right){
    //     BoundaryPoint point;
    //     point.x = it.x();
    //     point.y = it.y();
    //     point.s = it.s();
    //     right_boundary.push_back(point);
    // }

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

int main(int argc, char ** argv){
    std::vector<ExPathPoint> path_in;
    std::vector<BoundaryPoint> left_boundary;                
    std::vector<BoundaryPoint> right_boundary;
    std::vector<ExPathPoint> smooth_path_out;
    std::vector<xVector2d> Trajectory_from_tda4;

    Get_Offline_Data(path_in, left_boundary, right_boundary, Trajectory_from_tda4);

    // auto res = PathProcess::PathSmooth(path_in, left_boundary, right_boundary, smooth_path_out);
    std::vector<std::vector<xVector2d>> ALL_intersection;
    auto res = ExPathsmoothprocess(path_in, left_boundary, right_boundary, smooth_path_out, ALL_intersection);
    std::cout << "smoother.status():" << res << std::endl;

//画图---------------------------------------------------
    std::vector<xVector2d> obs_comefrom_Semanticline;
    size_t minSize = std::min(left_boundary.size(), right_boundary.size());

    for (size_t i = 0; i < minSize; ++i) {
        obs_comefrom_Semanticline.push_back(xVector2d(left_boundary[i].x, left_boundary[i].y));
        obs_comefrom_Semanticline.push_back(xVector2d(right_boundary[i].x, right_boundary[i].y));
    }
    for (size_t i = minSize; i < left_boundary.size(); ++i) {
        obs_comefrom_Semanticline.push_back(xVector2d(left_boundary[i].x, left_boundary[i].y));
    }
    for (size_t i = minSize; i < right_boundary.size(); ++i) {
        obs_comefrom_Semanticline.push_back(xVector2d(right_boundary[i].x, right_boundary[i].y));
    }
    std::vector<xVector2d> data_point_in;
    for(const ExPathPoint& it : path_in){
        data_point_in.push_back(xVector2d(it.x, it.y));
    }
    std::vector<xVector2d> data_point_new;
    sample_data(data_point_in, 1, data_point_new);

//判断转弯，把直线段拉直
//------------------------------------------------
    double Cur;  //判断转弯阈值
    std::vector<int> Corners_Ip;
    std::vector<xVector2d> corners_points;
    for (size_t i = 0; i < data_point_new.size(); i++) {
        // 计算每个点曲率，筛选出转弯点记录ip,设定提前(延后)2个数值点(4m)进入(离开)弯道
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
//------------------------------------------------

    namespace plt = matplotlibcpp;
    std::vector<double> x_s, y_s;

    // x_s.clear();
    // y_s.clear();
    // for (std::size_t j = 0; j < Ori_Path_Data.size(); j++) {
    //     x_s.emplace_back(Ori_Path_Data[j].x());
    //     y_s.emplace_back(Ori_Path_Data[j].y());
    // }
    // plt::plot(x_s, y_s, "b-");

// 检查转弯点
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < data_point_new.size(); j++) {
        x_s.emplace_back(data_point_new[j].x());
        y_s.emplace_back(data_point_new[j].y());
    }
    plt::plot(x_s, y_s);
    x_s.clear();
    y_s.clear();
    for (std::size_t j = 0; j < corners_points.size(); j++) {
        x_s.emplace_back(corners_points[j].x());
        y_s.emplace_back(corners_points[j].y());
    }
    plt::scatter(x_s, y_s, 15);
    plt::axis("equal");
    plt::show();

    // namespace plt = matplotlibcpp;
    double scale = 700;
    auto w = static_cast<int>(4 * scale);
    auto h = static_cast<int>(3 * scale);
    plt::figure_size(w, h);

plt::subplot(2, 2, 1);
    x_s.clear();
    y_s.clear();
    std::vector<double> vehx;
    std::vector<double> vehy;
    for (std::size_t i = 0; i < smooth_path_out.size(); i++) {
        // auto xy = opt_spline.Evaluate(0, t_s[i]);
        auto xy = xVector2d(smooth_path_out[i].x, smooth_path_out[i].y);
        x_s.emplace_back(xy.x());
        y_s.emplace_back(xy.y());
        // auto heading_xy = normalizeVectorxVector2d(opt_spline.Evaluate(1, t_s[i]));
        auto heading_xy = normalizeVectorxVector2d(xVector2d(cos(smooth_path_out[i].theta), sin(smooth_path_out[i].theta)));
        auto heng_xy = xVector2d(heading_xy.y(), -heading_xy.x());  //朝右

        auto veh11 = xy + (-0.940 * heading_xy) + 1.5 * heng_xy;
        auto veh22 = xy + (-0.940 * heading_xy) - 1.5 * heng_xy;
        auto veh33 = xy + 4.096 * heading_xy + 1.5 * heng_xy;
        auto veh44 = xy + 4.096 * heading_xy - 1.5 * heng_xy;   

        vehy.push_back(veh11.y());
        vehy.push_back(veh22.y());
        vehy.push_back(veh44.y());
        vehy.push_back(veh33.y());
        vehy.push_back(veh11.y());

        vehx.push_back(veh11.x());
        vehx.push_back(veh22.x());
        vehx.push_back(veh44.x());
        vehx.push_back(veh33.x());
        vehx.push_back(veh11.x());
        plt::plot(vehx, vehy, "y");
        vehy.clear();
        vehx.clear();


        auto veh1 = xy + (-0.940 * heading_xy) + 1.1 * heng_xy;
        auto veh2 = xy + (-0.940 * heading_xy) - 1.1 * heng_xy;
        auto veh3 = xy + 4.096 * heading_xy + 1.1 * heng_xy;
        auto veh4 = xy + 4.096 * heading_xy - 1.1 * heng_xy;   

        vehy.push_back(veh1.y());
        vehy.push_back(veh2.y());
        vehy.push_back(veh4.y());
        vehy.push_back(veh3.y());
        vehy.push_back(veh1.y());

        vehx.push_back(veh1.x());
        vehx.push_back(veh2.x());
        vehx.push_back(veh4.x());
        vehx.push_back(veh3.x());
        vehx.push_back(veh1.x());
        plt::plot(vehx, vehy, "g");
        vehy.clear();
        vehx.clear();


    }
    plt::plot(x_s, y_s, "b");

    // x_s.clear();
    // y_s.clear();
    // for (const auto &p : Trajectory_from_tda4) { //data_point_new data_point
    //     x_s.emplace_back(p.x());
    //     y_s.emplace_back(p.y());
    // }
    // plt::plot(x_s, y_s, "black");

    x_s.clear();
    y_s.clear();
    for (const auto &p : data_point_new) { //data_point_new data_point
        x_s.emplace_back(p.x());
        y_s.emplace_back(p.y());
    }
    plt::plot(x_s, y_s, "r");

    std::vector<double> x_obs;
    std::vector<double> y_obs;
    for(const xVector2d& obs : obs_comefrom_Semanticline){ 
        y_obs.push_back(obs.y());
        x_obs.push_back(obs.x());
    }
    plt::scatter(x_obs, y_obs);



    std::vector<xVector2d> control;
    std::vector<xVector2d> dddd;
// {data_point_new.begin() + 1, data_point_new.end() - 1}
    for (size_t i = 0; i < data_point_new.size()-2; i++) {
        control.push_back(data_point_new[i+1]);

        std::vector<double> xxx_s, yyy_s;
        xxx_s.clear();
        yyy_s.clear();
        if (ALL_intersection[i].empty()) {
            continue;
        }
        for (const auto &p : ALL_intersection[i]) { //data_point_new data_point
            xxx_s.emplace_back(p.x());
            yyy_s.emplace_back(p.y());
        }
        xxx_s.emplace_back(ALL_intersection[i][0].x());
        yyy_s.emplace_back(ALL_intersection[i][0].y());
        plt::plot(xxx_s, yyy_s, "r");

        x_s.clear();
        y_s.clear();
        dddd.clear();
        for (int k = 0; k < 12; k++) {
            xVector2d obs_helps;
            double angle = 2 * M_PI * k / 12;
            obs_helps.set_x(1.5 * cos(angle) + data_point_new[i+1].x());
            obs_helps.set_y(1.5 * sin(angle) + data_point_new[i+1].y());
            dddd.push_back(obs_helps);
        }
        for (std::size_t j = 0; j < dddd.size(); j++) {
            x_s.emplace_back(dddd[j].x());
            y_s.emplace_back(dddd[j].y());
        }
        x_s.emplace_back(dddd[0].x());
        y_s.emplace_back(dddd[0].y());
        plt::plot(x_s, y_s, "y--");

    }

    std::vector<double> x_control;
    std::vector<double> y_control;
    for(const xVector2d& obs : control){ 
        y_control.push_back(obs.y());
        x_control.push_back(obs.x());
    }
    plt::scatter(x_control, y_control, 20);


    plt::title("PC Optimize trajectory(blue)  original trajectory(red) tda4 Optimize trajectory(black)");
    plt::axis("equal");


plt::subplot(2, 2, 2);
    x_s.clear();
    y_s.clear();
    for (std::size_t i = 0; i < smooth_path_out.size(); i++) {
        auto xy = xVector2d(smooth_path_out[i].x, smooth_path_out[i].y);
        // auto xy = opt_spline.Evaluate(0, t_s[i]);
        x_s.emplace_back(xy.x());
        y_s.emplace_back(xy.y());
    }
    plt::plot(x_s, y_s, "b");

    x_s.clear();
    y_s.clear();
    for (const auto &p : Trajectory_from_tda4) { //data_point_new data_point
        x_s.emplace_back(p.x());
        y_s.emplace_back(p.y());
    }
    // plt::plot(x_s, y_s, "black");
    plt::title("PC Optimize trajectory(blue)   tda4 Optimize trajectory(black)");
    plt::axis("equal");


plt::subplot(2, 2, 3);
    x_s.clear();
    y_s.clear();
    for (size_t i = 0; i < smooth_path_out.size(); i++) {
        x_s.emplace_back(smooth_path_out[i].s);
        y_s.emplace_back(smooth_path_out[i].kappa);
    }
    plt::plot(x_s, y_s, {{"label", "kappa"}, {"linewidth", "0.5"}});
    x_s.clear();
    y_s.clear();
    for (size_t i = 0; i < smooth_path_out.size(); i++) {
        x_s.emplace_back(smooth_path_out[i].s);
        y_s.emplace_back(smooth_path_out[i].dkappa);
    }
    plt::plot(x_s, y_s, {{"label", "dkappa"}, {"linewidth", "0.5"}});

plt::subplot(2, 2, 4);
    x_s.clear();
    y_s.clear();
    for (size_t i = 0; i < smooth_path_out.size(); i++) {
        x_s.emplace_back(smooth_path_out[i].s);
        y_s.emplace_back(smooth_path_out[i].theta);
    }
    plt::plot(x_s, y_s, {{"label", "theta_s"}, {"linewidth", "0.5"}});

    plt::legend();
    plt::show();
    plt::savefig("output.svg");
//---------------------------------------------------------------------------
/*
cv::Mat image(4 * 500, 4 * 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // 定义4个子图的位置和大小
    cv::Rect subplot1(0, 0, 500, 500);
    cv::Rect subplot2(500, 0, 500, 500);
    cv::Rect subplot3(0, 500, 500, 500);
    cv::Rect subplot4(500, 500, 500, 500);

// subplot1
    cv::Mat subimage1 = image(subplot1);
    std::vector<cv::Point> xy_cv;
    std::vector<cv::Point> veh;
    for (Eigen::Index i = 0; i < t_s.size(); i++) {
        auto xy = opt_spline.Evaluate(0, t_s[i]);
        xy_cv.emplace_back(cv::Point(xy.x(), xy.y()));
        auto heading_xy = normalizeVectorxVector2d(opt_spline.Evaluate(1, t_s[i]));
        auto heng_xy = xVector2d(heading_xy.y(), -heading_xy.x());  //朝右

        auto veh11 = xy + (-0.940 * heading_xy) + 1.5 * heng_xy;
        auto veh22 = xy + (-0.940 * heading_xy) - 1.5 * heng_xy;
        auto veh33 = xy + 4.096 * heading_xy + 1.5 * heng_xy;
        auto veh44 = xy + 4.096 * heading_xy - 1.5 * heng_xy;   

        veh.emplace_back(cv::Point(veh11.x(), veh11.y()));
        veh.emplace_back(cv::Point(veh22.x(), veh22.y()));
        veh.emplace_back(cv::Point(veh44.x(), veh44.y()));
        veh.emplace_back(cv::Point(veh33.x(), veh33.y()));
        veh.emplace_back(cv::Point(veh11.x(), veh11.y()));
        cv::polylines(subimage1, veh, false, cv::Scalar(0, 255, 255), 2);

        veh.clear();

        auto veh1 = xy + (-0.940 * heading_xy) + 1.1 * heng_xy;
        auto veh2 = xy + (-0.940 * heading_xy) - 1.1 * heng_xy;
        auto veh3 = xy + 4.096 * heading_xy + 1.1 * heng_xy;
        auto veh4 = xy + 4.096 * heading_xy - 1.1 * heng_xy;   

        veh.emplace_back(cv::Point(veh1.x(), veh1.y()));
        veh.emplace_back(cv::Point(veh2.x(), veh2.y()));
        veh.emplace_back(cv::Point(veh4.x(), veh4.y()));
        veh.emplace_back(cv::Point(veh3.x(), veh3.y()));
        veh.emplace_back(cv::Point(veh1.x(), veh1.y()));
        cv::polylines(subimage1, veh, false, cv::Scalar(0, 255, 0), 2);
        veh.clear();
    }
    cv::polylines(subimage1, xy_cv, false, cv::Scalar(255, 0, 0), 2);

    xy_cv.clear();
    for (const auto &p : data_point_new) { //data_point_new data_point
        xy_cv.emplace_back(cv::Point(p.x(), p.y()));
    }
    cv::polylines(subimage1, xy_cv, false, cv::Scalar(0, 0, 255), 2);

    xy_cv.clear();
    for(const xVector2d& obs : obs_comefrom_Semanticline){ 
        xy_cv.emplace_back(cv::Point(obs.x(), obs.y()));
    }
    cv::circle(subimage1, xy_cv, 5, cv::Scalar(255, 0, 0), -1);

// subplot2
    cv::Mat subimage2 = image(subplot2);
    xy_cv.clear();
    for (Eigen::Index i = 0; i < t_s.size(); i++) {
        auto xy = opt_spline.Evaluate(0, t_s[i]);
        xy_cv.emplace_back(cv::Point(xy.x(), xy.y()));
    }
    cv::polylines(subimage2, xy_cv, false, cv::Scalar(255, 0, 0), 2);

    xy_cv.clear();
    for (const auto &p : Trajectory_from_tda4) { 
        xy_cv.emplace_back(cv::Point(p.x(), p.y()));
    }
    cv::polylines(subimage2, xy_cv, false, cv::Scalar(0, 0, 0), 2);

// subplot3
    cv::Mat subimage3 = image(subplot3);
    xy_cv.clear();
    for (size_t i = 0; i < s_s.size(); i++) {
        xy_cv.emplace_back(cv::Point(s_s[i], kappa_s[i]));
    }
    cv::polylines(subimage3, xy_cv, false, cv::Scalar(255, 0, 0), 2);

    xy_cv.clear();
    for (size_t i = 0; i < s_s.size(); i++) {
        xy_cv.emplace_back(cv::Point(s_s[i], dkappa_s[i]));
    }
    cv::polylines(subimage3, xy_cv, false, cv::Scalar(0, 255, 0), 2);

// subplot4
    cv::Mat subimage4 = image(subplot4);
    xy_cv.clear();
    for (size_t i = 0; i < s_s.size(); i++) {
        xy_cv.emplace_back(cv::Point(s_s[i], theta_s[i]));
    }
    cv::polylines(subimage3, xy_cv, false, cv::Scalar(0, 255, 0), 2);

    // cv::imshow("OpenCV Plot", image);
    cv::waitKey(0);
//---------------------------------------------------------------------
*/
// // 记录DataPoint
//     auto basis = opt_spline.Basis();
//     Eigen::VectorXd t_s = Eigen::VectorXd::LinSpaced(500, 0, 1);
//     std::ofstream file1("/log/hpa_routes/degug_result.csv");
//     if (!file1.is_open()) {
//         std::cerr << "Error opening file1: " << "degug_result.csv" << std::endl;
//         // return;
//     }
//     // 写入数据行
//     for (Eigen::Index i = 0; i < t_s.size(); i++) {
//         auto xy = opt_spline.Evaluate(0, t_s[i]);
//         file1 << xy.x();
//         file1 << ",";
//         file1 << xy.y();
//         file1 << "\n";
//     }
//     file1.close();
//---------------------------------------------------------------
}

