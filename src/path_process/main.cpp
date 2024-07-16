#include <cstdio>
#include <string>

#include "common.h"
#include "path_process_api.h"

std::string g_current_program_dir;

int main(int argc, char **argv) {
    std::vector<MapPosRecord> path;
    MapIO mapper;
    mapper.LoadMap("map1_2023-9-11-16-59-53.bin");
    bool have_path = mapper.GetMapPath(path);

    std::vector<ExPathPoint> pre_path_out;
    auto res = ExPathPreprocess(path, pre_path_out);
    std::cout << "Preprocess.status():" << res << std::endl;

    std::vector<ExPathPoint> pre_path_in;
    std::vector<ExPathPoint> smoothed_path;
    std::vector<BoundaryPoint> left_boundary;
    std::vector<BoundaryPoint> right_boundary;

    res = ExPathsmoothprocess(pre_path_in, left_boundary, right_boundary, smoothed_path);
    std::cout << "smoother.status():" << res << std::endl;

    printf("done!\n");
    // (void)have_path;
    (void)res;
    return 0;
}
