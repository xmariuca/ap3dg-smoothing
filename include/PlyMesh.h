#ifndef __PLY_CLASS__
#define __PLY_CLASS__

#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Sparse>
#include <vector>
#include "defs.h"
#include <Eigen/IterativeLinearSolvers>



namespace N3dCW2
{
    struct MyTraits: public OpenMesh::DefaultTraits
    {
        typedef OpenMesh::Vec3d Point;
        typedef OpenMesh::Vec3d Normal;
        VertexAttributes( OpenMesh::Attributes::Normal |
            OpenMesh::Attributes::Color
        );
        FaceAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color
        );
    };
    typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;

    class PlyMesh
    {
    public:
        static int meshCount;
        PlyMesh(std::string filename);
        PlyMesh(Eigen::MatrixXd& pPointCloud);
        PlyMesh(const PlyMesh&);
        ~PlyMesh();
        PlyMesh& operator= (const PlyMesh &pSrc);
        bool writeMesh(std::string filename);
        void setColour(OpenMesh::Vec3uc colour);
        void addNoise(double noiseStd);
        void setColour(Eigen::VectorXd& vals);
        void getVecticesE(Eigen::MatrixXd& outVertMat);
        void setVerticesE(Eigen::MatrixXd newVertPos);
        // void getULTriplets(std::vector<Triplet>& tripletLaplacian, std::vector<Triplet>& tripletValence);
        // void getScalingTriplets(std::vector<Triplet>& tripletScaling);
        // void getUniformLaplaceBeltrami(Eigen::MatrixXd& uniformLB);
        void computeGaussianCurvature(Eigen::VectorXd& gaussCurv);
        void computeMeanCurvature_UL(Eigen::VectorXd& meanCurv);
        void computeMeanCurvature_LB(Eigen::VectorXd& meanCurv);
        void getPrincipalCurvatures(Eigen::VectorXd& meanCurv, Eigen::VectorXd& gaussCurv, Eigen::MatrixXd& out_princCurv);
        void computeExplicitSmoothingUL(double lambda, int numIterations);
        void computeImplicitSmoothingUL(double lambda, int numIterations);
        uint32_t getTotalVertices() { return m_vertNum;}

    private:
        MyMesh m_mesh;
        uint32_t m_vertNum;
    };

}//namespace N3dCW2
#endif
