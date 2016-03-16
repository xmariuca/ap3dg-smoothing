#ifndef __CONVERTER__
#define __CONVERTER__

#include <Eigen/Dense>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <stdio.h>
#include <stdlib.h>     /* malloc, free, rand */


namespace N3dCW2
{

Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord);

OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord);


}//namespace N3dCW2
#endif
