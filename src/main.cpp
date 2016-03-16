// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
// basic file operations
#include <fstream>
#include <iostream>
#include <string>
#include "PlyMesh.h"
#include "utils.h"
#include "defs.h"
#include <Eigen/Dense>
using namespace N3dCW2;

// ***************************************************************************
int main(int argc, const char * argv[])
{
    Eigen::Matrix3d rotationMat(Eigen::Matrix3d::Zero());

    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_3_FILENAME);
    // mesh_M1.computeNormals();

    // size_t numVerts = mesh_M1.getTotalVertices();
    // Eigen::MatrixXd unifLaplacian(numVerts,numVerts);

    mesh_M1.computeUniformLaplacian();
    // printMessage("all good");

    return 0;
}
