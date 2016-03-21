#include "utils.h"
#include <iostream>
#include "defs.h"
#include <OpenMesh/Core/Utils/color_cast.hh>



namespace N3dCW2
{
    // ************************************************
    void printMessage(std::string text)
    {
        std::cout<< text << std::endl;
    }
    // ************************************************
    void getColourFromList(int idx,OpenMesh::Vec3uc& outColour)
    {
        OpenMesh::Vec3uc purple(92,75,81);
        OpenMesh::Vec3uc lightBlue(140,190,178);
        OpenMesh::Vec3uc lightYellow(242,235,191);
        OpenMesh::Vec3uc lightOrange(243,181,98);
        OpenMesh::Vec3uc lightPink(240,96,96);
        OpenMesh::Vec3uc otherBlue(95,172,190);

        outColour = otherBlue;
        if (idx == 1 ) outColour = purple;
        if (idx == 2 ) outColour = lightBlue;
        if (idx == 3 ) outColour = lightYellow;
        if (idx == 4 ) outColour = lightOrange;
        if (idx == 5 ) outColour = lightPink;
        if (idx == 6 ) outColour = otherBlue;
    }
    // ************************************************
    OpenMesh::Vec3uc getColour(double v, double vmin, double vmax, double median)
    {
        double red = 0.0, green = 0.0, blue = 0.0;
        // map it into [0,1]
        double newV = (v - vmin)/(vmax - vmin);
        double zeroVal = (-vmin)/(vmax - vmin);
        double medianInRange = (median - vmin)/(vmax - vmin);

        double thresh = medianInRange/500;

        if (newV < (zeroVal - thresh))
        {
            blue = 1.0;
        }
        else if (newV > (zeroVal + thresh))
        {
            red = 1.0;
        }
        if( vmin == 0)
        { // zeroval = 0
            red = 0.0, green = 0.0, blue = 0.0;
            if (newV > (zeroVal + thresh))
            {
                red = 1.0;
            }
        }
        if( vmax == 0)
        {// zeroval = 1
            red = 0.0, green = 0.0, blue = 0.0;
            if (newV > (zeroVal - thresh))
            {
                blue = 1.0;
            }
        }
        OpenMesh::Vec3uc colourFinal((unsigned int)std::floor(red*255), (unsigned int)std::floor(green*255), (unsigned int)std::floor(blue*255));
        return colourFinal;
    }
    // ************************************************
    double getTriangleArea(Eigen::Vector3d& vec1, Eigen::Vector3d& vec2, Eigen::Vector3d& vec3)
    {
        Eigen::Vector3d edge1 = (vec2 - vec1);
        Eigen::Vector3d edge2 = (vec3 - vec1);
        double thisArea = (1.0/6.0)*((edge1.cross(edge2)).norm());
        return thisArea;
    }

}//namespace N3dCW2
