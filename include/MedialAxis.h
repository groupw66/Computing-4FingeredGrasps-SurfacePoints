#ifndef MEDIALAXIS_H
#define MEDIALAXIS_H
#include<vector>
#include<Eigen/Dense>
#include "Flann3D.h"
#include "NaiveNN3D.h"
#include "NearestNeighbor.h"
#include "ObjectSurfacePoints.h"
#include "BasicGeometry.hpp"


class MedialAxis
{
    public:
        MedialAxis();
        virtual ~MedialAxis();
        MedialAxis(ObjectSurfacePoints _osp, int nnMode=1){ setObject(_osp, nnMode); }
        int setObject(ObjectSurfacePoints _osp, int nnMode=1);
        int genMedialPoint(std::tuple<Eigen::Vector3d, double, double> &medialPoint, Eigen::Vector3d &pointB,
                                       int iA=-1, int iB=-1, int randBMode=0, double minradius=0);
        int genMedialPoint(std::tuple<Eigen::Vector3d, double, double> &medialPoint, Eigen::Vector3d &pointB,
                                       Eigen::Vector3d pA_, Eigen::Vector3d nA_, Eigen::Vector3d pB_, double minradius=0);
        int genMedialPoints(std::vector< std::tuple<Eigen::Vector3d, double, double> > &medialPoints, int randBMode=0, double minradius=0);

        double computeRadius(Eigen::Vector3d pA, Eigen::Vector3d nA, Eigen::Vector3d pB);

        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Vector3d> normals;
        Flann3D flann3D;
        NaiveNN3D naiveNN3D;
        NearestNeighbor *nn3D;
    protected:
    private:
};

#endif // MEDIALAXIS_H
