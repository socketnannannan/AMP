#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstring>
#include "common.h"
#include <eigen3/Eigen/Dense>

void read_csv_data(std::vector<xVector2d>& ldfgs_DataPoint, std::vector<Eigen::MatrixXd>& ldfgs_A_qian, std::vector<Eigen::MatrixXd>& ldfgs_A_hou, std::vector<Eigen::VectorXd>& ldfgs_b_qian, 
            std::vector<Eigen::VectorXd>& ldfgs_b_hou, std::vector<int>& ldfgs_qian_poly_numes, std::vector<int>& ldfgs_hou_poly_numes, std::vector<xVector2d>& ldfgs_corners_points){

    // std::vector<xVector2d> ldfgs_DataPoint;
    std::ifstream TempIns1("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/DataPoint.csv");
    std::string line1;
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
        ldfgs_DataPoint.emplace_back(right_xx, right_yy);
    }     



    // std::vector<Eigen::MatrixXd> ldfgs_A_qian;
    std::ifstream TempIns2("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/A_qian.csv");
    std::string line2;
    while (getline(TempIns2, line2)) {
        std::istringstream s(line2);       
        std::vector<std::string> fields2;  
        std::string field2;
        Eigen::MatrixXd ldfgs_A1(80,2);
        while (getline(s, field2, ','))  
        {
            fields2.push_back(field2); 
        }
        for(int j=0; j<80; j++){
            ldfgs_A1(j,0) = std::stod(fields2[2*j]);
            ldfgs_A1(j,1) = std::stod(fields2[2*j+1]);
        }
        ldfgs_A_qian.emplace_back(ldfgs_A1);
    }  

    // std::vector<Eigen::MatrixXd> ldfgs_A_hou;
    std::ifstream TempIns3("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/A_hou.csv");
    std::string line3;
    while (getline(TempIns3, line3)) {
        std::istringstream s(line3);       
        std::vector<std::string> fields3;  
        std::string field3;
        Eigen::MatrixXd ldfgs_A2(80,2);
        while (getline(s, field3, ','))  
        {
            fields3.push_back(field3); 
        }
        for(int j=0; j<80; j++){
            ldfgs_A2(j,0) = std::stod(fields3[2*j]);
            ldfgs_A2(j,1) = std::stod(fields3[2*j+1]);
        }
        ldfgs_A_hou.emplace_back(ldfgs_A2);
    }  

    // std::vector<Eigen::VectorXd> ldfgs_b_qian;
    std::ifstream TempIns4("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/b_qian.csv");
    std::string line4;
    while (getline(TempIns4, line4)) {
        std::istringstream s(line4);       
        std::vector<std::string> fields4;  
        std::string field4;
        Eigen::VectorXd ldfgs_b2(80);
        while (getline(s, field4, ','))  
        {
            fields4.push_back(field4); 
        }
        for(size_t j=0; j<80; j++){
            if (j < fields4.size()) {
                    ldfgs_b2(j) = std::stod(fields4[j]);
                } else {
                    std::cout << "csv不足80"  << std::endl;
                }
        }
        ldfgs_b_qian.emplace_back(ldfgs_b2);
    }  

    // std::vector<Eigen::VectorXd> ldfgs_b_hou;
    std::ifstream TempIns5("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/b_hou.csv");
    std::string line5;
    while (getline(TempIns5, line5)) {
        std::istringstream s(line5);       
        std::vector<std::string> fields5;  
        std::string field5;
        Eigen::VectorXd ldfgs_b1(80);
        while (getline(s, field5, ','))  
        {
            fields5.push_back(field5); 
        }
        for(int j=0; j<80; j++){
            ldfgs_b1(j) = std::stod(fields5[j]);
        }
        ldfgs_b_hou.emplace_back(ldfgs_b1);
    }  

    // std::vector<int> ldfgs_qian_poly_numes;
    std::ifstream TempIns6("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/qian_poly_numes.csv");
    std::string line6;
    while (getline(TempIns6, line6)) {
        std::istringstream s(line6);       
        std::vector<std::string> fields6;  
        std::string field6;
        while (getline(s, field6, ','))  
        {
            fields6.push_back(field6); 
        }
        ldfgs_qian_poly_numes.emplace_back(std::stod(fields6[0]));
    }   

    // std::vector<int> ldfgs_hou_poly_numes;
    std::ifstream TempIns7("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/hou_poly_numes.csv");
    std::string line7;
    while (getline(TempIns7, line7)) {
        std::istringstream s(line7);       
        std::vector<std::string> fields7;  
        std::string field7;
        while (getline(s, field7, ','))  
        {
            fields7.push_back(field7); 
        }
        ldfgs_hou_poly_numes.emplace_back(std::stod(fields7[0]));
    }  

    // std::vector<xVector2d> ldfgs_corners_points;
    std::ifstream TempIns8("/home/aibao/HiPhi/apps/map_io/offline_debug/data_front_optimization/11.2/hpa_routes/Get_Corners_Points.csv");
    std::string line8;
    while (getline(TempIns8, line8)) {
        std::istringstream s(line8);       
        std::vector<std::string> fields8;  
        std::string field8;
        while (getline(s, field8, ','))  
        {
            fields8.push_back(field8); 
        }
        double right_xx = std::stod(fields8[0]);
        double right_yy = std::stod(fields8[1]);
        ldfgs_corners_points.emplace_back(right_xx, right_yy);
    }     
}