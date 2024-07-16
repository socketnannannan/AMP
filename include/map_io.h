/**
 * @file map_io.h
 * @brief 地图读取和存储相关功能
 */

#ifndef HPA_SLAM_MAP_IO_H
#define HPA_SLAM_MAP_IO_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <dirent.h>
#include <memory>
#include <tuple>
#include <vector>
#define USE_ROS ///默认编译的为ROS版
#ifdef USE_DESAY
static std::string g_map_path = "/log/hpa_routes/"; ///tda4路径接
#elif defined USE_ROS
#include <ros/ros.h>
#include <ros/package.h>
static std::string g_map_path = ros::package::getPath ("hpa_view") + "/map/hpa/"; ///ros获取路径接口
#else
static std::string g_map_path = "~/maps/"; ///define your own path dir 
#endif

struct Point2dCov{
    Eigen::Vector2f point{0, 0};
    Eigen::Vector2f std{0, 0};
};

typedef std::pair<Point2dCov, Point2dCov> Points2dPair;

struct Point3dCov{
    Eigen::Vector3f point{0, 0, 0};
    Eigen::Vector3f std{0, 0, 0};
    inline Point2dCov to_2d()const{
        Point2dCov point2d_cov = Point2dCov();
        point2d_cov.point = point.head(2);
        point2d_cov.std = std.head(2);
        return point2d_cov;
    }
};

typedef std::pair<Point3dCov, Point3dCov> Points3dPair;

struct SemanticPoint{
    Point3dCov p;
    int flag{};
};

/**
 * @brief 经纬度点
 */
struct GpsPose {
    Eigen::Vector3d P{0, 0, 0};//lat, lon, alt
    Eigen::Quaternionf Q{1, 0, 0, 0};
};
/**
 * @brief 地图轨迹点
 * 地图轨迹点位置和姿态，如需添加速度和档位等信息可以评论提需求
 */
struct MapPose{
    Eigen::Vector3f P{0, 0, 0};
    Eigen::Quaternionf Q{1, 0, 0, 0};

    //同一个坐标系运算
    friend MapPose operator+(const MapPose& p1, const MapPose& p2);
    friend MapPose operator-(const MapPose& p1, const MapPose& p2);

    MapPose operator+=(const MapPose& p);
    MapPose Slerp(const MapPose &p, float scale)const;
};

/**
 * @brief 地图轨迹点
 * 地图轨迹点位置和姿态，如需添加速度和档位等信息可以评论提需求
 */
struct MapPosCov{
    MapPose pose;
    Eigen::MatrixXf cov;
};

struct MapPosRecord{
    MapPose pose;
    int direction{};///档位
};

typedef std::pair<double, MapPose> MapPosStamped;
typedef std::pair<double, MapPosCov> MapPosCovStamped;

/**
 * @brief 语义目标物
 * 语义信息集合
 */
struct SemanticPolygon{
    int id{}, type{}, flag{};
    MapPose center;
    std::vector<SemanticPoint> points;
};

struct MapHeader{
    int id{-1};
    std::string map_name;
    GpsPose map_lla;//lat, lon, alt, att
    MapPose start_p;
    MapPose end_p;
};

enum path_status {
    VALID,
    KAPPA_OUT_OF_RANGE_IN_PREPROCESS,
    CANNOT_FIND_START_SNIPPET,
    CANNOT_FIND_END_SNIPPET,
    HANDLE_REVERSE_FAILED,
    SAMPLING_FAILED,
    SMOOTH_FAILED,
    NOT_ENOUGH_CONTROL_POINT,
    PREPROCESSED_PATH_TOO_SHORT,
    PREPROCESSED_PATH_TOO_LONG,
    KAPPA_OUT_OF_RANGE_IN_SMOOTH,
};

struct BoundaryPoint{
    double x{};
    double y{};
    double s{};

    BoundaryPoint(double _x, double _y, double _s) : 
        x(_x), y(_y), s(_s) {}

    BoundaryPoint() : x(0.0), y(0.0), s(0.0) {}
};

struct ExPathPoint {
    double x{};
    double y{};
    double z{};
    double theta{};
    double kappa{};
    double s{};
    double dkappa{};

    ExPathPoint(double _x, double _y, double _z, double _theta, double _kappa, double _s, double _dkappa) : 
        x(_x), y(_y), z(_z), theta(_theta), kappa(_kappa), s(_s), dkappa(_dkappa) {}

    ExPathPoint() : x(0.0), y(0.0), z(0.0), theta(0.0), kappa(0.0), s(0.0), dkappa(0.0) {}
};

/**
 * @brief 地图读取和存储的io
 */
class MapIO {
public:
    MapIO() = default;

    ~MapIO(){
        if(map_file_.is_open()) {
            map_file_.close();
        }
    }

    /**
     * @brief 载入地图
     * @param map_name
     * @return
     */
    bool LoadMap(const std::string& map_name);

    /**
     * @brief 获取地图的头
     * @param map_header
     * @return
     */
    bool GetMapHeader(MapHeader &map_header);

    /**
     * @brief 载入地图轨迹点
     * @param path: 学习的轨迹点
     * @return 是否载入成功
     */
    bool GetMapPath(std::vector<MapPosRecord>& path);

    /**
     * @brief 载入语义地图
     * @param semantic_marks 语义目标物集合
     * @return 是否载入成功
     */
    bool GetSemanticMap(std::vector<SemanticPolygon>& semantic_marks);


    /**
     * @brief
     * @param map_file
     * @param header
     * @param path
     * @param semantic_marks
     * @return
     */
    static bool WriteMap(std::ofstream &map_file,
                         const MapHeader &header,
                         const std::vector<MapPosRecord> &path,
                         const std::vector<SemanticPolygon> &semantic_marks);
    /**
     * @brief
     * @param map_file
     * @param smoothed_path
     * @return
     */
    static bool WritePlanningMap(std::string &map_name,
                                 const std::vector<ExPathPoint>& smoothed_path);
    /**
     * @brief
     * @param map_file
     * @param smoothed_path
     * @return
     */
    static bool ReadPlanningMap(std::string &map_name,
                                std::vector<ExPathPoint>& smoothed_path);

    /**
     * @brief
     * @tparam Derived
     * @param R
     * @return
     */
    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> R2ypr(const Eigen::MatrixBase<Derived> &R) {
        auto n = R.col(0);
        auto o = R.col(1);
        auto a = R.col(2);

        Eigen::Matrix<typename Derived::Scalar, 3, 1> ypr(3);
        auto y = atan2(n(1), n(0));
        auto p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        auto r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr;
    }


private:

    /**
     * @brief map_file_跳转到读取规划轨迹的起始位置
     */
    void Go2PathPos();

    /**
     * @brief map_file_跳转到读取语义地图的起始位置
     */
    void Go2MapPos();

    template<typename Derived>
    void ReadP(Eigen::MatrixBase<Derived> &p){
        read(p.x());
        read(p.y());
        read(p.z());
    }

    void ReadPose(MapPose& pos){
        ReadP(pos.P);
        read(pos.Q.x());
        read(pos.Q.y());
        read(pos.Q.z());
        read(pos.Q.w());
    }

    void ReadPose(GpsPose& pos){
        ReadP(pos.P);
        read(pos.Q.x());
        read(pos.Q.y());
        read(pos.Q.z());
        read(pos.Q.w());
    }

    template<typename Derived>
    static void WriteP(std::ofstream& file, const Eigen::MatrixBase<Derived> &p){
        write(file, p.x());
        write(file, p.y());
        write(file, p.z());
    }

    static void WritePose(std::ofstream& file, const MapPose& pos){
        WriteP(file, pos.P);
        write(file, pos.Q.x());
        write(file, pos.Q.y());
        write(file, pos.Q.z());
        write(file, pos.Q.w());
    }

    static void WritePose(std::ofstream& file, const GpsPose& pos){
        WriteP(file, pos.P);
        write(file, pos.Q.x());
        write(file, pos.Q.y());
        write(file, pos.Q.z());
        write(file, pos.Q.w());
    }

    template<typename T>
    inline void read(T& data) {
        map_file_.read((char *) &data, sizeof(data));
    }

    template<typename T>
    inline static void write(std::ofstream& file, T const& data){
        file.write((char *) &data, sizeof(data));
    }


    inline void read_str(std::string& str){
        size_t str_size;
        read(str_size);
        char* buffer = new char[str_size];
        map_file_.read(buffer, str_size);
        str = buffer;
    }

    inline static void write_str(std::ofstream& file, const std::string& str){
        size_t str_size = str.size() + 1;
        write(file, str_size);
        file.write(str.c_str(), str_size);
    }

    std::ifstream map_file_;
    std::streampos header_pos_, path_pos_, map_pos_;
};


#endif //HPA_SLAM_MAP_IO_H
