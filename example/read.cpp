#include "map_log.h"

int main(int argc, char ** argv){
    MapIO map_io;
    std::string map_name = argv[1];
    if(map_io.LoadMap(map_name)) {
        MapHeader map_header;
        if (map_io.GetMapHeader(map_header)) {
            LogMapHeader(map_header);
        } else {
            std::cerr << "get map header error!" << std::endl;
            return 0;
        }

        std::vector<MapPosRecord> path;
        if (map_io.GetMapPath(path)) {
            LogMapPath(path);
        } else {
            std::cerr << "get map path error!" << std::endl;
            return 0;
        }
        std::vector<SemanticPolygon> semantic_marks;
        if (map_io.GetSemanticMap(semantic_marks)) {
            LogSemanticMap(semantic_marks);
        } else {
            std::cerr << "get semantic map error!" << std::endl;
            return 0;
        }
    }
    else {
        std::cerr << "load map error!" << std::endl;
        return 0;
    }
}