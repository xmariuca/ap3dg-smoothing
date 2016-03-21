#ifndef _H_DEF
#define _H_DEF
#include <string>
#include <Eigen/Sparse>
// #include <vector>

namespace N3dCW2
{
    #define SUCCESS 0
    #define ERROR 1


    typedef Eigen::Triplet<double> Triplet;
    typedef Eigen::SparseMatrix<double,Eigen::RowMajor> SparseMat;

    static std::string BASE_PATH = "../src/plyMeshes/";
    static std::string EXTENSION = ".ply";

    static std::string BUNNY_MESH = "../src/plyMeshes/_bunny.ply";
    // static std::string BUNNY_MESH_NOISY = "../src/plyMeshes/noisy.ply";

    static std::string DRAGON_MESH = "../src/plyMeshes/_dragon.ply";
    static std::string CUBE_MESH = "../src/plyMeshes/_cube.ply";
    static std::string ARMADILLO_MESH = "../src/plyMeshes/_armadillo.ply";
}
#endif
