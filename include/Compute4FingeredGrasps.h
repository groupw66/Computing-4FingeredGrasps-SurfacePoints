#ifndef COMPUTE4FINGEREDGRASPS_H
#define COMPUTE4FINGEREDGRASPS_H

#include <Eigen/Dense>
#include <vector>
#include "SurfacePoint.h"
#include "Grasp.h"
#include "BasicGeometry.hpp"

namespace Compute4FingeredGrasps
{
	std::vector<std::vector<Grasp> > Compute4FingeredGrasps(const std::vector<SurfacePoint> &surfacePoints,
                                               const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle=10.d);

    std::vector<SurfacePoint> pointInConesFilter(const std::vector<SurfacePoint> &surfacePoints,
                                               Eigen::Vector3d point, double halfAngle=10.d);

    std::vector<Grasp> findEquilibriumGrasps_naive(const std::vector<SurfacePoint> &M, Eigen::Vector3d samplePoint);

    bool isEquilibriumGrasp(Grasp grasp, Eigen::Vector3d point);

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
};

#endif // COMPUTE4FINGEREDGRASPS_H
