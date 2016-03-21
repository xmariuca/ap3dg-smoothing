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
// ************************************************
// TASK 1
// ************************************************
void DiscreteCurvature(std::string mesh_filename)
{
    printMessage("reading ply mesh ...");
    PlyMesh mesh_M1(mesh_filename);

    // ************************************************
    printMessage("get mean curvature - uniform laplacian...");
    Eigen::VectorXd meanCurvUL;
    mesh_M1.computeMeanCurvature_UL(meanCurvUL);
    mesh_M1.setColour(meanCurvUL);
    std::string filenameUL = BASE_PATH + std::string("meanCurv_ul") + EXTENSION;
    mesh_M1.writeMesh(filenameUL);

    // ************************************************
    printMessage("get mean curvature - laplace beltrami...");
    Eigen::VectorXd meanCurvLB;
    mesh_M1.computeMeanCurvature_LB(meanCurvLB);
    mesh_M1.setColour(meanCurvLB);
    std::string filenameLB = BASE_PATH + std::string("meanCurv_lb") + EXTENSION;
    mesh_M1.writeMesh(filenameLB);

    // ************************************************
    printMessage("get gaussian curvature...");
    Eigen::VectorXd gaussCurv;
    mesh_M1.computeGaussianCurvature(gaussCurv);
    mesh_M1.setColour(gaussCurv);
    std::string filenameGauss = BASE_PATH + std::string("gaussCurv") + EXTENSION;
    mesh_M1.writeMesh(filenameGauss);

    if(1)
    {
        // ************************************************
        printMessage("get principal curvatures - uniform laplacian...");
        Eigen::MatrixXd princCurv_ul;
        // uniform laplacian
        mesh_M1.getPrincipalCurvatures(meanCurvUL, gaussCurv, princCurv_ul);
        Eigen::VectorXd k1_ul(princCurv_ul.row(0));
        Eigen::VectorXd k2_ul(princCurv_ul.row(1));
        // principal curvature k1_ul
        mesh_M1.setColour(k1_ul);
        std::string filenameK1_ul = BASE_PATH + std::string("princCurv_k1_ul") + EXTENSION;
        mesh_M1.writeMesh(filenameK1_ul);
        // principal curvature k2_ul
        mesh_M1.setColour(k2_ul);
        std::string filenameK2_ul = BASE_PATH + std::string("princCurv_k2_ul") + EXTENSION;
        mesh_M1.writeMesh(filenameK2_ul);

        // ************************************************
        printMessage("get principal curvatures - laplace beltrami...");
        Eigen::MatrixXd princCurv_lb;
        // laplace beltrami
        mesh_M1.getPrincipalCurvatures(meanCurvLB, gaussCurv, princCurv_lb);
        Eigen::VectorXd k1_lb(princCurv_lb.row(0));
        Eigen::VectorXd k2_lb(princCurv_lb.row(1));
        // principal curvature k1_lb
        mesh_M1.setColour(k1_lb);
        std::string filenameK1_lb = BASE_PATH + std::string("princCurv_k1_lb") + EXTENSION;
        mesh_M1.writeMesh(filenameK1_lb);
        // principal curvature k2_lb
        mesh_M1.setColour(k2_lb);
        std::string filenameK2_lb = BASE_PATH + std::string("princCurv_k2_lb") + EXTENSION;
        mesh_M1.writeMesh(filenameK2_lb);
    }


    printMessage("Done!");
}


// // ************************************************
// TASK 2
// ************************************************
static double const LAMBDA_EXPLICIT = 1;
static int const NUM_ITERATIONS_EX = 2;

static double const LAMBDA_IMPLICIT = 3;
static int const NUM_ITERATIONS_IM = 1;


void LaplacianSmoothing(std::string mesh_filename)
{
    printMessage("compute explicit smoothing ...");
    printMessage("reading ply mesh ...");
    PlyMesh mesh_M1(mesh_filename);
    OpenMesh::Vec3uc outColourES;
    getColourFromList(4,outColourES);
    mesh_M1.setColour(outColourES);
    printMessage("smooth mesh ...");
    mesh_M1.computeExplicitSmoothingUL(LAMBDA_EXPLICIT,NUM_ITERATIONS_EX);
    printMessage("write output to file ...");
    std::string filenameExplSmoothUL = BASE_PATH + std::string("explicitSmoothing_ul") + EXTENSION;
    mesh_M1.writeMesh(filenameExplSmoothUL);

    printMessage("compute implicit smoothing ...");
    printMessage("reading ply mesh ...");
    PlyMesh mesh_M2(mesh_filename);
    OpenMesh::Vec3uc outColourIS;
    getColourFromList(5,outColourIS);
    mesh_M2.setColour(outColourIS);
    printMessage("smooth mesh ...");
    mesh_M2.computeImplicitSmoothingUL(LAMBDA_IMPLICIT,NUM_ITERATIONS_IM);
    printMessage("write output to file ...");
    std::string filenameImplSmoothUL = BASE_PATH + std::string("implicitSmoothing_ul") + EXTENSION;
    mesh_M2.writeMesh(filenameImplSmoothUL);

    printMessage("Done!");

}

void NoiseExperiments(std::string mesh_filename)
{
    double lambda_ex = 0.7;
    int numIter_ex = 5;

    double lambda_im = 2.0;
    int numIter_im = 5;

    OpenMesh::Vec3uc outColourD;

    printMessage("reading ply mesh ...");
    PlyMesh mesh_M1(mesh_filename);
    getColourFromList(4,outColourD);
    mesh_M1.setColour(outColourD);
    printMessage("smooth mesh - explicit ...");
    mesh_M1.computeExplicitSmoothingUL(lambda_ex, numIter_ex);
    printMessage("write output to file ...");
    std::string filenameNExpl = BASE_PATH + std::string("noisy_exSm") + EXTENSION;
    mesh_M1.writeMesh(filenameNExpl);

    printMessage("reading ply mesh ...");
    PlyMesh mesh_M2(mesh_filename);
    getColourFromList(5,outColourD);
    mesh_M2.setColour(outColourD);
    printMessage("smooth mesh - implicit ...");
    mesh_M2.computeImplicitSmoothingUL(lambda_im, numIter_im);
    printMessage("write output to file ...");
    std::string filenameNImpl = BASE_PATH + std::string("noisy_imSm") + EXTENSION;
    mesh_M2.writeMesh(filenameNImpl);

    printMessage("Done!");
}
int main(int argc, const char * argv[])
{
    printMessage("\n*** \nDiscrete curvature \n***\n");
    // BUNNY_MESH, DRAGON_MESH, CUBE_MESH, ARMADILLO_MESH
    DiscreteCurvature(DRAGON_MESH);

    printMessage("\n*** \nLaplacian Mesh Smoothing \n***\n");
    LaplacianSmoothing(CUBE_MESH);

    printMessage("\n*** \nNoise Experiments \n***\n");
    OpenMesh::Vec3uc outColourD;
    PlyMesh mesh_M1(DRAGON_MESH);
    mesh_M1.addNoise(0.001);
    getColourFromList(2,outColourD);
    mesh_M1.setColour(outColourD);
    printMessage("write output to file ...");
    std::string filenameNoisy = BASE_PATH + std::string("dragon_noisy") + EXTENSION;
    mesh_M1.writeMesh(filenameNoisy);

    NoiseExperiments(filenameNoisy);

    return SUCCESS;
}
