#include "Grasp.h"

Grasp::Grasp()
{
    //ctor
}

Grasp::~Grasp()
{
    //dtor
}

Grasp::Grasp(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4)
{
    surfacePoints.clear();
    surfacePoints.push_back(sp1);
    surfacePoints.push_back(sp2);
    surfacePoints.push_back(sp3);
    surfacePoints.push_back(sp4);
    std::sort(surfacePoints.begin(), surfacePoints.end());
    //cm = _cm;
}

Grasp::Grasp(const std::vector<SurfacePoint>& _surfacePoints)
{
    surfacePoints.clear();
    surfacePoints.insert(surfacePoints.end(), _surfacePoints.begin(), _surfacePoints.end());
    std::sort(surfacePoints.begin(), surfacePoints.end());
    //cm = _cm;
}
