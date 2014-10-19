#include "Grasp.h"

Grasp::Grasp()
{
    //ctor
}

Grasp::~Grasp()
{
    //dtor
}

Grasp::Grasp(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4, Eigen::Vector3d _cm)
{
    surfacePoints.clear();
    surfacePoints.insert(sp1);
    surfacePoints.insert(sp2);
    surfacePoints.insert(sp3);
    surfacePoints.insert(sp4);
    cm = _cm;
}

Grasp::Grasp(const std::vector<SurfacePoint>& _surfacePoints, Eigen::Vector3d _cm)
{
    surfacePoints.clear();
    surfacePoints.insert(_surfacePoints.begin(), _surfacePoints.end());
    cm = _cm;
}
