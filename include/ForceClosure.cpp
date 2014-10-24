#include "ForceClosure.h"

double ForceClosure::getMindist_Qhull(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4, Eigen::Vector3d cm, double halfAngle, int nPyramidSide)
{
    std::vector<Wrench> wrenchs;
    std::vector<Wrench> tmpWrenchs;
    tmpWrenchs = sp1.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp2.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp3.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp4.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    return getMindist_Qhull(wrenchs);
}

double ForceClosure::getMindist_Qhull(std::vector<Wrench> wrenchs)
{
    double mindist = std::numeric_limits<double>::max();
    orgQhull::Qhull qhull;
    realT points[6 * wrenchs.size()];
    for(unsigned int i=0 ; i < wrenchs.size() ; ++i){
        points[i*6] = wrenchs[i](0);
        points[i*6+1] = wrenchs[i](1);
        points[i*6+2] = wrenchs[i](2);
        points[i*6+3] = wrenchs[i](3);
        points[i*6+4] = wrenchs[i](4);
        points[i*6+5] = wrenchs[i](5);
    }
    qhull.runQhull("", 6, wrenchs.size(), points, "");
    std::vector<orgQhull::QhullFacet> facets=qhull.facetList().toStdVector();
    for(const orgQhull::QhullFacet &facet : qhull.facetList().toStdVector()){
        mindist = std::min(mindist, -facet.getFacetT()->offset);
    }
    return mindist;
}


