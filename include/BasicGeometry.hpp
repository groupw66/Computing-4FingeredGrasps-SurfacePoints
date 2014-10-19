#ifndef BASICGEOMETRY_H
#define BASICGEOMETRY_H

#include <Eigen/Dense>
#include <algorithm>
#include <Qhull.h>
#include <QhullFacetList.h>

using namespace orgQhull;

namespace Geometry
{
    inline double angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b)
    {
        double tmp = a.dot(b)/(a.norm()*b.norm());
        tmp = std::max( std::min(tmp, 1.0d), -1.0d);
        return acos(tmp);
    }

    inline bool isVectorsPositivelySpan3DQhull(const std::vector<Eigen::Vector3d> &vectors)
    {
        Qhull qhull;
        realT points[3 * vectors.size()];
        for(int i=0 ; i < vectors.size() ; ++i){
            points[i*3] = vectors[i].x();
            points[i*3+1] = vectors[i].y();
            points[i*3+2] = vectors[i].z();
        }
        qhull.runQhull("", 3, vectors.size(), points, "");
        std::vector<QhullFacet> facets=qhull.facetList().toStdVector();
        for(const QhullFacet &facet : qhull.facetList().toStdVector())
            if(facet.getFacetT()->offset>=0) return false;
        return true;
    }

    inline bool isVectorsPositivelySpan3D(Eigen::Vector3d n1, Eigen::Vector3d n2,
                                          Eigen::Vector3d n3, Eigen::Vector3d n4)
    {
        Eigen::Vector3d p1 = n2.cross(n3);
        Eigen::Vector3d p2 = n1.cross(n3);
        Eigen::Vector3d p3 = n1.cross(n2);
        return p1.dot(n4)*p1.dot(n1)<0 && p2.dot(n4)*p2.dot(n2)<0 && p3.dot(n4)*p3.dot(n3)<0 ;
    }

    inline bool isVectorsPositivelySpan3D(const std::vector<Eigen::Vector3d> &vectors)
    {
        if(vectors.size() <= 3)
            return false;
        else if(vectors.size() == 4)
            return isVectorsPositivelySpan3D(vectors[0], vectors[1], vectors[2], vectors[3]);
        else
            return isVectorsPositivelySpan3DQhull(vectors);
    }
}
#endif // BASICGEOMETRY_H
