#ifndef __UTILS__
#define __UTILS__

#include <string>
#include <Eigen/Dense>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


namespace N3dCW2
{
    void printMessage(std::string text);
    OpenMesh::Vec3uc getColour(double v, double vmin, double vmax, double median);
    double getTriangleArea(Eigen::Vector3d& vec1, Eigen::Vector3d& vec2, Eigen::Vector3d& vec3);
    void getColourFromList(int idx,OpenMesh::Vec3uc& outColour);

}//namespace N3dCW2

#endif
