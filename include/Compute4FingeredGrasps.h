#ifndef COMPUTE4FINGEREDGRASPS_H
#define COMPUTE4FINGEREDGRASPS_H

#include <Eigen/Dense>
#include <vector>
#include "SurfacePoint.h"
#include "Grasp.h"
#include "BasicGeometry.hpp"

namespace Compute4FingeredGrasps
{
	void compute4FingeredGrasps(std::vector<std::vector<Grasp> > &sol, const std::vector<SurfacePoint> &surfacePoints,
                                               const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle=10.d);

    void pointInConesFilter(std::vector<SurfacePoint> &filtereds, const std::vector<SurfacePoint> &surfacePoints,
                                               Eigen::Vector3d point, double halfAngle=10.d);

    void findEquilibriumGrasps_naive(std::vector<Grasp>  &sol, const std::vector<SurfacePoint> &M, Eigen::Vector3d samplePoint);

    bool isEquilibriumGrasp(Grasp grasp, Eigen::Vector3d point);

    inline void uniqueSol(std::vector<Grasp>  &uniqueGrasps, const std::vector<std::vector<Grasp> > &allGrasps)
    {
        uniqueGrasps.clear();
        std::set<Grasp> setGrasps;
        for(std::vector<Grasp> grasps : allGrasps){
            setGrasps.insert(grasps.begin(), grasps.end());
        }
        uniqueGrasps.insert(uniqueGrasps.end(), setGrasps.begin(), setGrasps.end());
    }

    inline bool isPointInCone(Eigen::Vector3d point, SurfacePoint surfacePoint, double halfAngle=10.d)
    {
        double angle = halfAngle*M_PI/180.d;
        Eigen::Vector3d v = point - surfacePoint.position;
        double angleV = Geometry::angleBetweenVectors(surfacePoint.normal, v);
        return angleV < angle && angleV >= 0;
    }

    inline bool isPointInConeDoubleside(Eigen::Vector3d point, SurfacePoint surfacePoint, double halfAngle=10.d)
    {
        SurfacePoint surfacePoint2(surfacePoint.position, -surfacePoint.normal);
        return isPointInCone(point, surfacePoint, halfAngle) || isPointInCone(point, surfacePoint2, halfAngle);
    }

    inline Eigen::Vector3d findVectorInward(Eigen::Vector3d point, Eigen::Vector3d position, Eigen::Vector3d normal)
    {
        Eigen::Vector3d v = position - point;
        if(Geometry::angleBetweenVectors(v, normal) < 0){
            v = -v;
        }
        return v;
    }
};

#endif // COMPUTE4FINGEREDGRASPS_H