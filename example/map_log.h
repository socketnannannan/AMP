/**
 * @file map_io.h
 * @brief 地图读取和存储相关功能
 */

#ifndef MAP_IO_MAP_LOG_H
#define MAP_IO_MAP_LOG_H

// #ifdef M_PI
    # define M_PI		3.14159265358979323846	/* pi */
// #endif

#include "map_io.h"

void LogQ(const Eigen::Quaternionf &q) {
  std::cout << "  q: x y z w " << q.x() << " " << q.y() << " " << q.z() << " "
            << q.w() << std::endl;
  std::cout << "  yaw pitch roll: "
            << MapIO::R2ypr(q.toRotationMatrix()).transpose() * 180 / M_PI
            << std::endl;
}

void LogMapHeader(const MapHeader &map_header) {
  std::cout << "-------------map header------------" << std::endl;
  std::cout << "id: " << map_header.id << " ";
  std::cout << "name: " << map_header.map_name << std::endl;
  std::cout << "map lat lon alt: " << std::endl;
  auto &p = map_header.map_lla.P;
  printf("%.7f %.7f %.2f", p.x(), p.y(), p.z());
  std::cout << "map attitude: " << std::endl;
  LogQ(map_header.map_lla.Q);
  std::cout << "map start p: " << std::endl;
  std::cout << "  " << map_header.start_p.P.transpose() << std::endl;
  std::cout << "map start attitude: " << std::endl;
  LogQ(map_header.start_p.Q);
  std::cout << "map end p: " << std::endl;
  std::cout << "  " << map_header.end_p.P.transpose() << std::endl;
  std::cout << "map end attitude: " << std::endl;
  LogQ(map_header.end_p.Q);
}

void LogMapPath(const std::vector<MapPosRecord> &path) {
  std::cout << "-------------map path------------" << std::endl;
  std::cout << "path points num: " << path.size() << std::endl;
  for (const auto &p : path) {
    std::cout << p.pose.P.transpose() << std::endl;
    LogQ(p.pose.Q);
    std::cout << " direction(-1: backward, 0: standstill, 1: forward): "
              << p.direction << std::endl;
  }
}

void LogSemanticMap(const std::vector<SemanticPolygon> &semantic_marks) {
  std::cout << "-------------semantic map------------" << std::endl;
  std::cout << "semantic mark num: " << semantic_marks.size() << std::endl;
  for (const auto &sm : semantic_marks) {
    std::cout << "--id: " << sm.id << " type: " << sm.type
              << " flag: " << sm.flag << std::endl;
    std::cout << "  center: " << sm.center.P.transpose() << std::endl;
    LogQ(sm.center.Q);
    std::cout << "  point size: " << sm.points.size()
              << "............................" << std::endl;
    for (const auto &sp : sm.points) {
      std::cout << "  point: " << sp.p.point.transpose() << std::endl;
      std::cout << "  std, flag: " << sp.p.std.transpose() << " " << sp.flag
                << std::endl;
    }
  }
}

#endif // MAP_IO_MAP_LOG_H
