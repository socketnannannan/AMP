#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstring>
#include "common.h"
#include <eigen3/Eigen/Dense>
void write_data(const std::vector<xVector2d>& DataPoint, const std::vector<Eigen::MatrixXd>& A_front, const std::vector<Eigen::VectorXd> &b_front, const std::vector<Eigen::MatrixXd> &A_rear, 
                const std::vector<Eigen::VectorXd> &b_rear, const std::vector<int> &front_poly_numes, const std::vector<int> &rear_poly_numes, const std::vector<xVector2d> &Corners_Points){
// 记录DataPoint
// /log/hpa_routes/
// /log/hpa_routes/
    std::ofstream file1("/log/hpa_routes/plan1point.csv");
    if (!file1.is_open()) {
        std::cerr << "Error opening file1: " << "DataPoint().csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < DataPoint.size(); ++i) {
        file1 << DataPoint[i].x();
        file1 << ",";
        file1 << DataPoint[i].y();
        file1 << "\n";
    }
    file1.close();




//记录A_front
    std::ofstream file2("/log/hpa_routes/A_front.csv");
    if (!file2.is_open()) {
        std::cerr << "Error opening file2: " << "A_front.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < A_front.size(); ++i) {
        for(int j=0; j<80; j++){
            if(fabs(A_front[i](j,0))<1.0e-6){
                file2 << 0.0;
                file2 << ",";
            }else{
                file2 << A_front[i](j,0);
                file2 << ",";
            }
            if(fabs(A_front[i](j,0))<1.0e-6){
                file2 << 0.0;
                file2 << ",";
            }else{
                file2 << A_front[i](j,1);
                file2 << ",";
            }
        }
        file2 << "\n";
    }
    file2.close();


//记录A_rear
    std::ofstream file3("/log/hpa_routes/A_rear.csv");
    if (!file3.is_open()) {
        std::cerr << "Error opening file3: " << "A_rear.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < A_rear.size(); ++i) {
        for(int j=0; j<80; j++){
            if(fabs(A_rear[i](j,0))<1.0e-6){
                file3 << 0.0;
                file3 << ",";
            }else{
                file3 << A_rear[i](j,0);
                file3 << ",";
            }
            if(fabs(A_rear[i](j,1))<1.0e-6){
                file3 << 0.0;
                file3 << ",";
            }else{
                file3 << A_rear[i](j,1);
                file3 << ",";
            }
        }
        file3 << "\n";
    }
    file3.close();

//记录b_front
    std::ofstream file4("/log/hpa_routes/b_front.csv");
    if (!file4.is_open()) {
        std::cerr << "Error opening file4: " << "b_front.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < b_front.size(); ++i) {
        for(int j=0; j<80; j++){
            if(fabs(b_front[i](j))<1.0e-6){
                file4 << 0.0;
                file4 << ",";
            }else{
                file4 << b_front[i](j);
                file4 << ",";
            }
        }
        file4 << "\n";
    }
    file4.close();

//记录b_rear
    std::ofstream file5("/log/hpa_routes/b_rear.csv");
    if (!file5.is_open()) {
        std::cerr << "Error opening file5: " << "b_rear.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < b_rear.size(); ++i) {
        for(int j=0; j<80; j++){
            if(fabs(b_rear[i](j))<1.0e-6){
                file5 << 0.0;
                file5 << ",";
            }else{
                file5 << b_rear[i](j);
                file5 << ",";
            }
        }
        file5 << "\n";
    }
    file5.close();

//记录front_poly_numes
    std::ofstream file6("/log/hpa_routes/front_poly_numes.csv");
    if (!file6.is_open()) {
        std::cerr << "Error opening file6: " << "front_poly_numes.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < front_poly_numes.size(); ++i) {
        if(fabs(front_poly_numes[i])<1.0e-6){
            file6 << 0.0;
            file6 << "\n";
        }else{
            file6 << front_poly_numes[i];
            file6 << "\n";
        }
    }
    file6.close();    

//记录rear_poly_numes
    std::ofstream file7("/log/hpa_routes/rear_poly_numes.csv");
    if (!file7.is_open()) {
        std::cerr << "Error opening file7: " << "rear_poly_numes.csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < rear_poly_numes.size(); ++i) {
        if(fabs(rear_poly_numes[i])<1.0e-6){
            file7 << 0.0;
            file7 << "\n";
        }else{
            file7 << rear_poly_numes[i];
            file7 << "\n";
        }
    }
    file7.close();  

//记录Corners_Points
    std::ofstream file8("/log/hpa_routes/Get_Corners_Points.csv");
    if (!file8.is_open()) {
        std::cerr << "Error opening file8: " << "Get_Corners_Points().csv" << std::endl;
        // return;
    }
    // 写入数据行
    for (size_t i = 0; i < Corners_Points.size(); ++i) {
        file8 << Corners_Points[i].x();
        file8 << ",";
        file8 << Corners_Points[i].y();
        file8 << "\n";
    }
    file8.close();  
}