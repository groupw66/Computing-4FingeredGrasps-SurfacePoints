#ifndef BASICGEOMETRY_H
#define BASICGEOMETRY_H

#include <Eigen/Dense>
#include <algorithm>

namespace Geometry
{
    static inline double angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b){
        double tmp = a.dot(b)/(a.norm()*b.norm());
        tmp = std::max( std::min(tmp, 1.0d), -1.0d);
        return acos(tmp);
    }
}
#endif // BASICGEOMETRY_H
