#include "map_log.h"
#include <random>

int main(int argc, char **argv) {
    std::ofstream map_file;
    std::string map_name = "test.bin";
    std::string map_file_path = g_map_path + map_name;
    map_file.open(map_file_path, std::ios::out | std::ios::trunc | std::ios::binary);
    MapHeader map_header;
    std::vector<MapPosRecord> path;
    std::vector<SemanticPolygon> semantic_marks;

    /**
     * 生成随机地图
     */
    std::default_random_engine e(time(nullptr));
    std::uniform_int_distribution<int> id(1, 254);

    map_header.id = id(e);
    map_header.map_name = map_name;
    map_header.map_lla.P.setRandom();
    map_header.map_lla.Q.setIdentity();
    map_header.start_p.P.setRandom();
    map_header.start_p.Q.setIdentity();
    map_header.end_p.P.setRandom();
    map_header.end_p.Q.setIdentity();

    std::uniform_int_distribution<size_t> path_size(500, 5000);///20cm 一个点，100m~1000m
    size_t points_size = path_size(e);
    for (size_t i = 0; i < points_size; i++) {
        MapPosRecord path_p{};
        path_p.pose.P.setRandom();
        path_p.pose.Q.setIdentity();
        path_p.direction = 1;
        path.emplace_back(std::move(path_p));
    }

    std::uniform_int_distribution<size_t> semantic_size(20, 100);///
    size_t slots_size = semantic_size(e);
    for (size_t i = 0; i < slots_size; i++) {
        SemanticPolygon sp;
        sp.id = i;
        sp.flag = 0;
        sp.type = 0;
        Eigen::Vector3f sum_pos{0, 0, 0};
        for (int j = 0; j < 4; j++) {
            SemanticPoint s_p{};
            s_p.p.point.setRandom();
            s_p.p.std.setZero();
            s_p.flag = 0;
            sum_pos += s_p.p.point;
            sp.points.emplace_back(std::move(s_p));
        }
        sum_pos /= 4;
        sp.center.P = sum_pos;
        sp.center.Q.setIdentity();
        semantic_marks.emplace_back(sp);
    }

    if (MapIO::WriteMap(map_file, map_header, path, semantic_marks)) {
        LogMapHeader(map_header);
        LogMapPath(path);
        LogSemanticMap(semantic_marks);
    } else {
        std::cerr << "save map failed!" << "map path" << g_map_path << " map name: " << map_name << std::endl;
    }


    return 0;
}