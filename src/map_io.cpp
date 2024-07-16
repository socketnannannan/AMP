#include "map_io.h"
#include "version.h"

int GetVersion() {
    int version{0};
    version += atoi(PROJECT_VER_MAJOR) * static_cast<int>(1e4);
    version += atoi(PROJECT_VER_MINOR) * static_cast<int>(1e2);
    version += atoi(PTOJECT_VER_PATCH);
    return version;
}

bool MapIO::LoadMap(const std::string &map_name) {
    std::string map_file_path = g_map_path + map_name;
    std::cout << "load map:" << map_file_path << std::endl;

    if (map_file_.is_open()) {
        map_file_.close();
    }

    map_file_.open(map_file_path, std::ios::binary);
    if (map_file_.bad()) {
        std::cerr << "map file error happened!" << std::endl;
        return false;
    }
    int map_version;
    read(map_version);
    int mapio_version = GetVersion();
    if (map_version/10000 != mapio_version/10000) {
        std::cerr << "The map version doesn't fit with the program, map version is: "
                  << map_version << " program version is: " << GetVersion()<<std::endl;
        return false;
    }
    else{
        std::cout<<"map version: "<<map_version<<std::endl;
    }
    header_pos_ = map_file_.tellg();
    return true;
}

bool MapIO::GetMapHeader(MapHeader &map_header) {
    if (!map_file_.is_open()) {
        std::cerr << "Load map first." << std::endl;
        return false;
    }

    read(map_header.id);
    read_str(map_header.map_name);
    ReadPose(map_header.map_lla);
    ReadPose(map_header.start_p);
    ReadPose(map_header.end_p);

    path_pos_ = map_file_.tellg(); ///记录读取轨迹起点的位置
    map_file_.clear();
    map_file_.seekg(header_pos_);
    return true;
}

bool MapIO::GetMapPath(std::vector<MapPosRecord> &path) {
    if (!map_file_.is_open()) {
        std::cerr << "Load map first." << std::endl;
        return false;
    }

    if (path_pos_ == std::ios::beg) {
        Go2PathPos();
    } else {
        map_file_.seekg(path_pos_);
    }

    size_t size_ps;
    read(size_ps);
    path.reserve(size_ps);

    for (size_t i = 0; i < size_ps; i++) {
        path.emplace_back();
        ReadPose(path.back().pose);
        read(path.back().direction);
    }

    map_pos_ = map_file_.tellg();///记录读取语义地图起点的位置
    map_file_.clear();
    map_file_.seekg(header_pos_);
    return true;
}

bool MapIO::GetSemanticMap(std::vector<SemanticPolygon> &semantic_marks) {
    if (!map_file_.is_open()) {
        std::cerr << "Load map first." << std::endl;
        return false;
    }

    if (map_pos_ == std::ios::beg) {
        Go2MapPos();
    } else {
        map_file_.seekg(map_pos_);
    }

    size_t marks_size;
    read(marks_size);
    semantic_marks.reserve(marks_size);
    for (size_t i = 0; i < marks_size; i++) {
        SemanticPolygon sp;
        read(sp.id);
        read(sp.type);
        read(sp.flag);
        ReadPose(sp.center);
        size_t points_size;
        read(points_size);

        for (size_t j = 0; j < points_size; j++) {
            sp.points.emplace_back();
            ReadP(sp.points.back().p.point);
            ReadP(sp.points.back().p.std);
            read(sp.points.back().flag);
        }
        semantic_marks.emplace_back(std::move(sp));
    }
    map_pos_ = map_file_.tellg();
    map_file_.clear();
    map_file_.seekg(header_pos_);
    return true;
}

bool MapIO::WriteMap(std::ofstream &map_file,
                     const MapHeader &header,
                     const std::vector<MapPosRecord> &path,
                     const std::vector<SemanticPolygon> &semantic_marks) {
    if(!map_file.is_open()){
        std::cerr << "map file does not init." << std::endl;
        return false;
    }
    int version = GetVersion();
    std::cout<<"map version: "<<version<<std::endl;
    write(map_file, version);
    /*存储header*/
    write(map_file, header.id);
    write_str(map_file, header.map_name);
    WritePose(map_file, header.map_lla);
    WritePose(map_file, header.start_p);
    WritePose(map_file, header.end_p);

    /*存储轨迹*/
    write(map_file, path.size());
    for (const auto &pp:path) {
        WritePose(map_file, pp.pose);
        write(map_file, pp.direction);
    }

    /*存储语义地图*/
    write(map_file, semantic_marks.size());
    for (const auto &mark:semantic_marks) {
        write(map_file, mark.id);
        write(map_file, mark.type);
        write(map_file, mark.flag);
        WritePose(map_file, mark.center);
        write(map_file, mark.points.size());

        for (const auto &p:mark.points) {
            WriteP(map_file, p.p.point);
            WriteP(map_file, p.p.std);
            write(map_file, p.flag);
        }
    }
    return true;
}

bool MapIO::WritePlanningMap(std::string &map_name,
                                 const std::vector<ExPathPoint>& smoothed_path){
    std::string csv_path = g_map_path + map_name;
    // LOG(INFO) << "csv path: " << csv_path;
    std::fstream f_map_table(csv_path, std::ios::out | std::ios::trunc);
    f_map_table     << "x" << ','
                    << "y" << ','
                    << "z" << ','
                    << "theta" << ','
                    << "kappa" << ','
                    << "s" << ','
                    << "dkappa"
                    << std::endl;
    for (const auto & path_point:smoothed_path) {
        f_map_table << path_point.x << ','
                    << path_point.y << ','
                    << path_point.z << ','
                    << path_point.theta << ','
                    << path_point.kappa << ','
                    << path_point.s << ','
                    << path_point.dkappa
                    << std::endl;
    }
    return true;
}

bool MapIO::ReadPlanningMap(std::string &map_name,
                                std::vector<ExPathPoint>& smoothed_path){
    std::string csv_path = g_map_path + map_name;
    std::fstream f_map_csv_;
    f_map_csv_.open(csv_path, std::ios::in | std::ios::out);
    if (!f_map_csv_.is_open()) {
        std::cout << "csv file is not exist: " << csv_path;
        return false;
    }
    std::istringstream sin;         //将整行字符串line读入到字符串istringstream中
    std::vector<std::string> words; //声明一个字符串向量
    std::string line, word;
    // 读取标题行
    std::getline(f_map_csv_, line);
    // 读取数据
    while (std::getline(f_map_csv_, line)) {
        ExPathPoint path_point;
        sin.clear();
        sin.str(line);
        words.clear();
        int pos{0};
        while (std::getline(sin, word, ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
        {
            switch (pos) {
                case 0:
                    path_point.x = strtod(word.data(), nullptr);
                    break;
                case 1:
                    path_point.y = strtod(word.data(), nullptr);
                    break;
                case 2:
                    path_point.z = strtod(word.data(), nullptr);
                    break;
                case 3:
                    path_point.theta = strtod(word.data(), nullptr);
                    break;
                case 4:
                    path_point.kappa = strtod(word.data(), nullptr);
                    break;
                case 5:
                    path_point.s = strtod(word.data(), nullptr);
                    break;
                case 6:
                    path_point.dkappa = strtod(word.data(), nullptr);
                    break;
                default:
                    break;
            }
            pos++;
        }
        smoothed_path.emplace_back(std::move(path_point));
    }

    return true;
}

void MapIO::Go2PathPos() {
    MapHeader map_header;
    read(map_header.id);
    read_str(map_header.map_name);
    size_t size_header{0};
    size_t lla_size = sizeof(map_header.map_lla.P) + sizeof(map_header.map_lla.Q);
    size_header += lla_size;
    size_t start_p_size = sizeof(map_header.start_p.P) + sizeof(map_header.start_p.Q);
    size_header += start_p_size;
    size_t end_p_size = sizeof(map_header.end_p.P) + sizeof(map_header.end_p.Q);
    size_header += end_p_size;
    map_file_.ignore(size_header); ///ignore map header
}

void MapIO::Go2MapPos() {
    Go2PathPos();
    size_t size_ps;
    read(size_ps);
    MapPosRecord path_p;
    size_t pos_rec_size = sizeof(path_p.pose.P) + sizeof(path_p.pose.Q) + sizeof(path_p.direction);
    map_file_.ignore(pos_rec_size * size_ps);
}

MapPose operator+(const MapPose &p1, const MapPose &p2) {
    MapPose p;
    p.P = p1.P + p2.P;
    p.Q = p1.Q * p2.Q;
    return p;
}

MapPose operator-(const MapPose &p1, const MapPose &p2) {
    MapPose p;
    p.P = p1.P - p2.P;
    p.Q = p1.Q * p2.Q.inverse();
    return p;
}

MapPose MapPose::operator+=(const MapPose &p) {
    P = P + p.P;
    Q = Q * p.Q;
    return *this;
}

MapPose MapPose::Slerp(const MapPose &p, float scale) const {
    MapPose p1;
    p1.P = P + (p.P - P) * scale;
    p1.Q = Q.slerp(scale, p.Q);
    return p1;
}