#ifndef FORCECLOSURE_H
#define FORCECLOSURE_H

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <Qhull.h>
#include <QhullFacetList.h>
#include "Wrench.h"
#include "SurfacePoint.h"

namespace ForceClosure
{
    double getMindist_Qhull(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4,
                      Eigen::Vector3d cm = Eigen::Vector3d(0,0,0), double halfAngle = 10.d, int nPyramidSide = DEFAULT_N_PYRAMID_SIDE);
    double getMindist_Qhull(std::vector<Wrench> wrenchs);
    //double getMindist_ZC(std::vector<Wrench> wrenchs);
	double getMindist_ZC(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4,
                      Eigen::Vector3d cm = Eigen::Vector3d(0,0,0), double halfAngle = 10.);
};

#endif // FORCECLOSURE_H
