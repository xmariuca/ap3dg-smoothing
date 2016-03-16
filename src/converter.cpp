#include "converter.h"

namespace N3dCW2
{

    Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord)
    {
        Eigen::Vector3d eCoord(omCoord[0],omCoord[1],omCoord[2]);
        return eCoord;
    }

    OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord)
    {
        OpenMesh::Vec3d omCoord(eCoord[0],eCoord[1],eCoord[2]);
        return omCoord;
    }
}//namespace N3dCW2
