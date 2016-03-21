#include <iostream>
#include  <Eigen/Dense>
#include "PlyMesh.h"
#include "converter.h"
#include "utils.h"
#include <random>
#include <math.h>       /* sqrt */


namespace N3dCW2
{
    int PlyMesh::meshCount = 0;
    // ****************************************************************
    PlyMesh::PlyMesh(std::string filename){
        OpenMesh::IO::Options ropt;
        ropt += OpenMesh::IO::Options::VertexColor;
        if (m_mesh.has_vertex_normals())
        {
            ropt += OpenMesh::IO::Options::VertexNormal;
            // ropt += OpenMesh::IO::Options::FaceColor;
        }
        if ( ! OpenMesh::IO::read_mesh(m_mesh, filename, ropt) )
        {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
        // If the file did not provide vertex normals, then calculate them
        if ( !ropt.check( OpenMesh::IO::Options::VertexNormal ) )
        {
            // we need face normals to update the vertex normals
            m_mesh.request_face_normals();

            // let the mesh update the normals
            m_mesh.update_normals();

            // dispose the face normals, as we don't need them anymore
            m_mesh.release_face_normals();
        }
        else
        {
            m_mesh.request_vertex_normals();
        }
        // just check if it really works
        if (m_mesh.has_vertex_normals())
        {
            std::cout << "computed the vertex normals\n";
        }

        m_vertNum = m_mesh.n_vertices();
        std::cout << "finished reading  " << m_vertNum << " vertices\n";
        meshCount++;
    }
    PlyMesh::PlyMesh(Eigen::MatrixXd& pPointCloud)
    {
        m_vertNum = pPointCloud.cols();
        // MyMesh::VertexHandle* vhandle = new MyMesh::VertexHandle[m_vertNum];

        for (int i = 0; i < m_vertNum; i++)
        {
            OpenMesh::Vec3d thisVert = convertEIGENVecToOMVec(pPointCloud.col(i));
            m_mesh.add_vertex(thisVert);
        }
        meshCount++;
    }
    // ****************************************************************
    PlyMesh::PlyMesh(const PlyMesh& pSrc)
    {
        m_mesh = pSrc.m_mesh;
        m_vertNum = pSrc.m_vertNum;
        meshCount++;
    }
    // ****************************************************************
    PlyMesh& PlyMesh::operator= (const PlyMesh& pSrc)
    {
        // check for self - assignment
        if (this == &pSrc)
        return *this;
        m_mesh = pSrc.m_mesh;
        m_vertNum = pSrc.m_vertNum;
        meshCount++;
        return *this;
    }
    // ****************************************************************
    PlyMesh::~PlyMesh(){}
    // ****************************************************************
    void PlyMesh::setColour(OpenMesh::Vec3uc colour)
    {
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            m_mesh.set_color(*v_it,colour);
        }
    }
    // ****************************************************************
    bool PlyMesh::writeMesh(std::string filename){
        OpenMesh::IO::Options wopt;
        wopt += OpenMesh::IO::Options::VertexColor;
        if (m_mesh.has_vertex_normals())
        {
            wopt += OpenMesh::IO::Options::VertexNormal;
            // wopt += OpenMesh::IO::Options::FaceColor;
        }
        if ( ! OpenMesh::IO::write_mesh(m_mesh, filename, wopt) )
        {
            std::cerr << "Error: cannot write mesh to " << filename << std::endl;
            return ERROR;
        }
        return SUCCESS;
    }
    // ****************************************************************
    void PlyMesh::addNoise(double noiseStd)
    {
        double noiseMean = 0.0;
        std::default_random_engine generator;
        std::normal_distribution<double> gaussNoise(noiseMean,noiseStd);
        double thisRandomNoise;
        // add random noise to each vertex
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            thisRandomNoise = gaussNoise(generator);
            // OpenMesh::Vec3d thisVNoise(thisRandomNoise,thisRandomNoise,thisRandomNoise);
            OpenMesh::Vec3d thisNormal = m_mesh.normal(*v_it) ;

            OpenMesh::Vec3d thisOMVert = m_mesh.point(*v_it);
            thisOMVert = thisOMVert + thisRandomNoise * thisNormal;
            m_mesh.set_point( *v_it, thisOMVert);
        }
    }
    // ****************************************************************

    void PlyMesh::setColour(Eigen::VectorXd& vals){
        double min = vals.minCoeff();
        double max = vals.maxCoeff();
        double step = 0;//std::abs(max - min)/10;

        //find median
        std::vector<double> sortedVals;
        for (int i=0; i < vals.size(); i++)
        {
            sortedVals.push_back(vals(i));
        }

        double median;
        size_t size = sortedVals.size();

        sort(sortedVals.begin(), sortedVals.end());

        if (size  % 2 == 0)
        {
            median = (sortedVals[size / 2 - 1] + sortedVals[size / 2]) / 2;
        }
        else
        {
            median = sortedVals[size / 2];
        }
        // vals = ((vals.array() - min)/ (max-min)).matrix();
        // double zeroVal = (0 - min)/ (max-min);
        // median = (median - min)/ (max-min);
        // min = vals.minCoeff();
        // max = vals.maxCoeff();
        std::cout << "min = " << min << "\n";
        // std::cout << "zero = " << zeroVal << "\n";
        // std::cout << "median = " << median << "\n";
        std::cout << "max = " << max << "\n";

        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            uint32_t vi = (*v_it).idx();
            OpenMesh::Vec3uc colour = getColour(vals(vi),min+step,max-step,median);
            m_mesh.set_color(*v_it,colour);
        }
    }
    // ****************************************************************
    void PlyMesh::getVecticesE(Eigen::MatrixXd& outVertMat)
    {
        uint32_t colIdx = 0;
        outVertMat = Eigen::MatrixXd::Zero(3,m_vertNum);

        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            OpenMesh::Vec3d thisOMVert = m_mesh.point(*v_it);
            Eigen::Vector3d thisEVert = convertOMVecToEIGENVec(thisOMVert);
            // vectors are represented as matrices in Eigen
            // .col = block operation for matrices and arrays
            outVertMat.col(colIdx) = thisEVert;
            colIdx += 1;
        }
    }
    // ****************************************************************
    void PlyMesh::setVerticesE(Eigen::MatrixXd newVertPos)
    {
        uint32_t colIdx = 0;
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            Eigen::Vector3d thisEVert = newVertPos.col(colIdx);

            OpenMesh::Vec3d thisOMVert = convertEIGENVecToOMVec(thisEVert);
            m_mesh.set_point( *v_it, thisOMVert);
            colIdx += 1;
        }
    }
    // // ****************************************************************
    // void PlyMesh::getScalingTriplets(std::vector<Triplet>& tripletScaling)
    // {
    //     tripletScaling.reserve(m_vertNum);
    //
    //     for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
    //     v_it != m_mesh.vertices_end(); ++v_it)
    //     {
    //         uint32_t vi = (*v_it).idx();
    //         Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
    //         // circulate around the current vertex
    //         double viNeighArea = 0;
    //         std::vector<uint32_t> idxNeighVect;
    //         //// ***************
    //         //// circulate around the 1 - ring neighbours
    //         Eigen::Vector3d prevVjPoint(0,0,0);
    //         Eigen::Vector3d currentVjPoint(0,0,0);
    //         for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
    //         {
    //             uint32_t vj = (*vv_it).idx();
    //             idxNeighVect.push_back(vj);
    //
    //             currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
    //             // std::cout << prevVjPoint.transpose() << "\t" << currentVjPoint.transpose() << "\n";
    //             if(!prevVjPoint.isZero() && !currentVjPoint.isZero())
    //             {
    //                 viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
    //             }
    //             prevVjPoint = currentVjPoint;
    //         }
    //
    //         if (viNeighArea)
    //         {
    //             // get the last face
    //             MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it);
    //             currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
    //
    //             viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
    //             // std::cout << "viNeighArea = " << viNeighArea << "\n";
    //             // scalingMat(vi,vi) = 1.0/viNeighArea;
    //             // viNeighArea = viNeighArea/3.0f;
    //             tripletScaling.push_back(Triplet(vi, vi, 1.0));
    //             for (int i = 0; i < idxNeighVect.size(); i++)
    //             {
    //                 tripletScaling.push_back(Triplet(vi, idxNeighVect.at(i), 1.0/viNeighArea));
    //             }
    //         }
    //
    //     }
    // }
    // // ****************************************************************
    // void PlyMesh::getULTriplets(std::vector<Triplet>& tripletLaplacian, std::vector<Triplet>& tripletValence)
    // {
    //     tripletLaplacian.reserve(6 * m_vertNum);
    //     tripletValence.reserve(6 * m_vertNum);
    //     // int numIter = 0;
    //     for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
    //     v_it != m_mesh.vertices_end(); ++v_it)
    //     {
    //         uint32_t vi = (*v_it).idx();
    //         float valence = 0.0f;
    //         // std::vector<MyMesh::Point> neighVect;
    //         std::vector<uint32_t> idxNeighVect;
    //         // circulate around the current vertex
    //         for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
    //         {
    //             uint32_t vj = (*vv_it).idx();
    //             idxNeighVect.push_back(vj);
    //             tripletLaplacian.push_back(Triplet(vi, vj, 1.0));
    //         }
    //         valence = idxNeighVect.size();
    //         for (int i = 0; i < valence; i++)
    //         {
    //             tripletValence.push_back(Triplet(vi, idxNeighVect.at(i), 1.0/float(valence)));
    //         }
    //         tripletValence.push_back(Triplet(vi, vi, 1.0));
    //         tripletLaplacian.push_back(Triplet(vi, vi, -1.0));
    //     }
    // }
    // *************************************************************
    void PlyMesh::computeGaussianCurvature(Eigen::VectorXd& gaussCurv)
    {
        gaussCurv = Eigen::VectorXd::Zero(m_vertNum);

        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
            uint32_t vi = (*v_it).idx();
            float valence = 0.0f;
            // std::vector<MyMesh::Point> neighVect;
            // std::vector<uint32_t> idxNeighVect;
            // circulate around the current vertex
            double viNeighArea = 0;
            double angleSum = 0;
            uint32_t currVjIdx = 0, prevVjIdx = 0;
            // std::vector<Eigen::Vector3d> neighbourVect;
            //// ***************
            //// circulate around the 1 - ring neighbours
            Eigen::Vector3d prevVjPoint(0,0,0);
            Eigen::Vector3d currentVjPoint(0,0,0);
            for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
            {
                valence ++;
                currVjIdx = (*vv_it).idx();
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                // neighbourVect.push_back(currentVjPoint);
                // std::cout << prevVjPoint.transpose() << "\t" << currentVjPoint.transpose() << "\n";
                if(!prevVjPoint.isZero() && !currentVjPoint.isZero())
                {
                    viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
                    Eigen::Vector3d edge1 = (prevVjPoint - viPoint).normalized();
                    Eigen::Vector3d edge2 = (currentVjPoint - viPoint).normalized();


                    double angle = std::acos(edge1.dot(edge2));

                    angleSum += angle;
                }

                prevVjPoint = currentVjPoint;
                // prevVjIdx = currVjIdx;
            }
            // break;
            if (viNeighArea !=0)
            {
                // get the last face
                MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it);
                currVjIdx = (*vv_it).idx();
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));

                viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
                Eigen::Vector3d edge1 = (prevVjPoint - viPoint).normalized();
                Eigen::Vector3d edge2 = (currentVjPoint - viPoint).normalized();


                double angle = std::acos(edge1.dot(edge2));
                // angle = angle * 180.0 / M_PI;

                angleSum += angle;
            }
            // *************************
            // build the laplace beltrami operator

            // break;
            if(viNeighArea != 0 && valence != 0 )
            {

                gaussCurv(vi) = (2 * M_PI - angleSum) / viNeighArea;
            }
            else
            {
                gaussCurv(vi) = 0.0f;
            }
        }
        // std::cout << "gaussCurv \n" << gaussCurv.head(5).transpose() << "\n";

    }
    // *************************************************************
    void PlyMesh::computeMeanCurvature_UL(Eigen::VectorXd& meanCurv)
    {
        meanCurv = Eigen::VectorXd::Zero(m_vertNum);

        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            Eigen::Vector3d thisLaplacian(0,0,0);
            Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
            uint32_t vi = (*v_it).idx();
            float valence = 0.0f;
            // circulate around the current vertex
            double viNeighArea = 0;
            Eigen::Vector3d prevVjPoint(0,0,0);
            Eigen::Vector3d currentVjPoint(0,0,0);
            for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
            {
                valence ++;
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                thisLaplacian += (currentVjPoint - viPoint);
                if(!prevVjPoint.isZero() && !currentVjPoint.isZero())
                {
                    viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
                }
                prevVjPoint = currentVjPoint;
            }
            if (viNeighArea !=0)
            {
                // get the last face
                MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it);
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
            }
            if(valence!=0 && viNeighArea!=0)
            {
                Eigen::Vector3d thisNormal = convertOMVecToEIGENVec(m_mesh.normal(*v_it));

                thisLaplacian = thisLaplacian/float(valence);
                // thisLaplacian = thisLaplacian/(float(valence) * viNeighArea);
                meanCurv(vi) = 0.5f * thisLaplacian.dot(thisNormal);
                // meanCurv(vi) = - 0.5f * thisLaplacian.norm();

                // std::cout << "thisNorm " << vi << "\n" << thisNormal.transpose() << "\n";
                // std::cout << "thisLaplac " << vi << "\n" << - 0.5f * thisLaplacian.transpose() << "\n";
            }
            else
            {
                meanCurv(vi) = 0.0f;
            }
        }

    }
    // *************************************************************
    void PlyMesh::computeMeanCurvature_LB(Eigen::VectorXd& meanCurv)
    {
        meanCurv = Eigen::VectorXd::Zero(m_vertNum);

        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            Eigen::Vector3d thisLaplacBelt(0,0,0);
            Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
            uint32_t vi = (*v_it).idx();
            float valence = 0;
            // circulate around the current vertex
            double viNeighArea = 0;
            uint32_t currVjIdx = 0, prevVjIdx = 0;
            // first row = idx vj1
            // second row = idx vj2
            // third row = angle
            int idxMat = 0;
            Eigen::MatrixXd angleMat(Eigen::MatrixXd::Zero(3,30));
            //// ***************
            //// circulate around the 1 - ring neighbours
            Eigen::Vector3d prevVjPoint(0,0,0);
            Eigen::Vector3d currentVjPoint(0,0,0);
            for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
            {
                valence ++;
                currVjIdx = (*vv_it).idx();
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                if(!prevVjPoint.isZero() && !currentVjPoint.isZero())
                {
                    viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);
                    Eigen::Vector3d edge1 = (prevVjPoint - currentVjPoint).normalized();
                    Eigen::Vector3d edge2 = (viPoint - currentVjPoint).normalized();
                    Eigen::Vector3d edge3 = (viPoint - prevVjPoint).normalized();

                    double angle1 = edge1.dot(edge2) / ((edge1.cross(edge2)).norm());
                    edge1 = - edge1;
                    double angle2 = edge1.dot(edge3) / ((edge1.cross(edge3)).norm());

                    angleMat.col(idxMat++) = Eigen::Vector3d(prevVjIdx,currVjIdx,angle1);
                    angleMat.col(idxMat++) = Eigen::Vector3d(currVjIdx,prevVjIdx,angle2);
                }

                prevVjPoint = currentVjPoint;
                prevVjIdx = currVjIdx;
            }
            // break;
            if (viNeighArea !=0)
            {
                // get the last face
                MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it);
                currVjIdx = (*vv_it).idx();
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                viNeighArea += getTriangleArea(viPoint, prevVjPoint, currentVjPoint);

                Eigen::Vector3d edge1 = (prevVjPoint - currentVjPoint).normalized();
                Eigen::Vector3d edge2 = (viPoint - currentVjPoint).normalized();
                Eigen::Vector3d edge3 = (viPoint - prevVjPoint).normalized();


                // double angle1 = std::acos((edge1.dot(edge2))/(edge1.norm() * edge2.norm()));
                // double angle2 = std::acos(((-edge1).dot(edge3))/((-edge1).norm() * edge3.norm()));
                double angle1 = edge1.dot(edge2) / ((edge1.cross(edge2)).norm());
                edge1 = - edge1;
                double angle2 = edge1.dot(edge3) / ((edge1.cross(edge3)).norm());

                angleMat.col(idxMat++) = Eigen::Vector3d(prevVjIdx,currVjIdx,angle1);
                angleMat.col(idxMat++) = Eigen::Vector3d(currVjIdx,prevVjIdx,angle2);
                // std::cout << "viNeighArea = " << viNeighArea << "\n";
                // scalingMat(vi,vi) = 1.0/viNeighArea;
                // std::cout << "\n" << angleMat << "\n";
            }
            // *************************
            // build the laplace beltrami operator
            for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
            {
                currVjIdx = (*vv_it).idx();
                currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                double angle1 = 0, angle2 = 0;
                bool cot1 = false, cot2 = false;
                for(int i = 0; i < angleMat.cols() && angleMat(2,i) !=0; i++)
                {
                    if(angleMat(0,i) == currVjIdx)
                    {
                        if(cot1 == false)
                        {
                            angle1 = angleMat(2,i);
                            cot1 = true;
                        }
                        if(cot2 == false)
                        {
                            angle2 = angleMat(2,i);
                            cot2 = true;
                        }
                    }
                }
                if(angle1 != 0 || angle2 !=0)
                {
                    // double weightLB = 1.0/std::tan(angle1) + 1.0/std::tan(angle2);
                    double weightLB = angle1 + angle2;
                    thisLaplacBelt += weightLB * (currentVjPoint - viPoint);
                }
            }
            // break;
            if(viNeighArea != 0 && valence != 0 )
            {
                Eigen::Vector3d thisNormal = convertOMVecToEIGENVec(m_mesh.normal(*v_it));

                thisLaplacBelt = thisLaplacBelt/ (2 * viNeighArea);
                // thisLaplacBelt = thisLaplacBelt/ (2 * viNeighArea * float(valence));
                meanCurv(vi) = 0.5f * thisLaplacBelt.dot(thisNormal);
            }
            else
            {
                meanCurv(vi) = 0.0f;
            }
            // std::cout << "meanCurv \n" << meanCurv.transpose() << "\n";
            // tripletValence.push_back(Triplet(vi, vi, 1.0/valence));
            // tripletLaplacianBeltrami.push_back(Triplet(vi, vi, 1.0));
        }

    }
    // *************************************************************
    void PlyMesh::getPrincipalCurvatures(Eigen::VectorXd& meanCurv, Eigen::VectorXd& gaussCurv, Eigen::MatrixXd& out_princCurv)
    {
        out_princCurv = Eigen::MatrixXd::Zero(2,m_vertNum);
        double k1 = 0, k2 = 0, diff = 0, thisH = 0, thisK = 0;
        for (int i = 0; i < m_vertNum; i++)
        {
            thisH = meanCurv(i);
            thisK = gaussCurv(i);

            diff = thisH * thisH - thisK;
            // std::cout << "diff = " << diff << std::endl;
            if (diff <= 0)
            {
                k1 = k2 = 0;
            }
            else
            {
                k1 = thisH + std::sqrt(diff);
                k2 = thisH - std::sqrt(diff);
            }
            out_princCurv(0,i) = k1;
            out_princCurv(1,i) = k2;
        }
        // std::cout << "princCurv \n" << out_princCurv.block(0,0,2,5) << "\n";
    }
    // *************************************************************
    void PlyMesh::computeExplicitSmoothingUL(double lambda, int numIterations)
    {
        for( int iter = 0; iter < numIterations; iter ++)
        {
            std::cout << "iteration " << iter+1 << std::endl;
            for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
            v_it != m_mesh.vertices_end(); ++v_it)
            {
                Eigen::Vector3d thisLaplacian(0,0,0);
                Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
                uint32_t vi = (*v_it).idx();
                float valence = 0.0f;
                Eigen::Vector3d prevVjPoint(0,0,0);
                Eigen::Vector3d currentVjPoint(0,0,0);
                Eigen::Vector3d updatedViPoint(0,0,0);
                for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
                {
                    valence ++;
                    currentVjPoint = convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                    thisLaplacian += (currentVjPoint - viPoint);
                    prevVjPoint = currentVjPoint;
                }
                if(valence!=0)
                {
                    thisLaplacian = thisLaplacian/float(valence);
                    updatedViPoint = viPoint + lambda * thisLaplacian;
                    m_mesh.set_point(*v_it, convertEIGENVecToOMVec(updatedViPoint));
                    // std::cout << "thisNorm " << vi << "\n" << thisNormal.transpose() << "\n";
                    // std::cout << "thisLaplac " << vi << "\n" << - 0.5f * thisLaplacian.transpose() << "\n";
                }
            }
        }
    }
    // *************************************************************
    void PlyMesh::computeImplicitSmoothingUL(double lambda, int numIterations)
    {
        std::vector<Triplet> tripletLaplacian;
        tripletLaplacian.reserve(10 * m_vertNum);
        // int numIter = 0;
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            uint32_t vi = (*v_it).idx();
            Eigen::Vector3d viPoint = convertOMVecToEIGENVec(m_mesh.point(*v_it));
            float valence = 0.0f;
            double thisVal = 0.0;
            // std::vector<Eigen::Vector3d> neighVect;
            std::vector<uint32_t> idxNeighVect;
            // circulate around the current vertex
            for (MyMesh::VertexVertexIter vv_it = m_mesh.vv_iter(*v_it); vv_it; ++vv_it)
            {
                uint32_t vj = (*vv_it).idx();
                // neighVect.push_back(convertOMVecToEIGENVec(m_mesh.point(*vv_it));
                idxNeighVect.push_back(vj);
            }
            valence = idxNeighVect.size();
            if (valence !=0)
            {
                for( int i = 0; i < valence; i++ )
                {
                    thisVal = - lambda/float(valence);
                    tripletLaplacian.push_back(Triplet(vi, idxNeighVect.at(i), thisVal));
                }
                tripletLaplacian.push_back(Triplet(vi, vi, (1.0 + lambda)));
            }
        }
        SparseMat A(m_vertNum,m_vertNum);
        A.setFromTriplets(tripletLaplacian.begin(), tripletLaplacian.end());
        Eigen::BiCGSTAB<SparseMat> solver;
        solver.compute(A);

        Eigen::VectorXd x,y,z;
        Eigen::MatrixXd currentVertices, updatedVertices;
        updatedVertices = Eigen::MatrixXd::Zero(3,m_vertNum);
        PlyMesh::getVecticesE(currentVertices);

        for( int iter = 0; iter < numIterations; iter ++)
        {
            std::cout << "iteration " << iter+1 << std::endl;

            x = solver.solve(currentVertices.row(0).transpose());
            y = solver.solve(currentVertices.row(1).transpose());
            z = solver.solve(currentVertices.row(2).transpose());

            updatedVertices.row(0) = x.transpose();
            updatedVertices.row(1) = y.transpose();
            updatedVertices.row(2) = z.transpose();

            // std::cout << "#iterations:     " << solver.iterations() << std::endl;
            // std::cout << "#estimated error: " << solver.error()      << std::endl;

            currentVertices = updatedVertices;
        }
        PlyMesh::setVerticesE(updatedVertices);
    }
}//namespace N3dCW2
