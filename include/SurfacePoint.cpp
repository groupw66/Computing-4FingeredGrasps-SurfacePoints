#include "SurfacePoint.h"

SurfacePoint::SurfacePoint()
{
    //ctor
}

SurfacePoint::~SurfacePoint()
{
    //dtor
}

SurfacePoint::SurfacePoint(Eigen::Vector3d _position, Eigen::Vector3d _normal)
{
    position = _position;
    normal = _normal.normalized();
}
