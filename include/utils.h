#ifndef __UTILS__
#define __UTILS__

#include <string>
#include <Eigen/Dense>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


namespace N3dCW2
{
    void printMessage(std::string text);
    Eigen::Matrix4d loadTransformation(int transfNo);
    void getColourFromList(int idx,OpenMesh::Vec3uc& outColour);
    void createRotationMatrix(float rotX, float rotY, float rotZ, Eigen::Matrix3d& rotMat);

}//namespace N3dCW2

#endif
