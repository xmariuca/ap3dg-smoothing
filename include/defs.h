#ifndef _H_DEF
#define _H_DEF
#include <string>

namespace N3dCW2
{
    #define TRANSF00_45 1
    #define TRANSF00_90 2
    #define TRANSF00_180 3
    #define TRANSF00_270 4
    #define TRANSF00_315 5
    #define TRANSF_45 6

    static const int MESH_NUMBER = 2;
    static std::string BASE_PATH = "../src/plyMeshes/";
    static std::string EXTENSION = ".ply";

    static std::string MODEL_1_FILENAME = "../src/plyMeshes/_bunny.ply";
    static std::string MODEL_2_FILENAME = "../src/plyMeshes/_dragon.ply";
    static std::string MODEL_3_FILENAME = "../src/plyMeshes/_cube.ply";
}
#endif
