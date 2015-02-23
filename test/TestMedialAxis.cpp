#include <UnitTest++.h>
#include "OBJFile.h"
#include "ObjectSurfacePoints.h"
#include "MedialAxis.h"
#include "Flann3D.h"
#include "NaiveNN3D.h"
#include "Timer.h"
#include "BasicGeometry.hpp"

namespace
{

TEST(MedialAxis)
{
    Timer tmr;

    OBJFile obj("test/meshes/KIT/Amicelli_800.obj");
    ObjectSurfacePoints osp(obj);
    MedialAxis medialAxis(osp);

    Flann3D flann3D(medialAxis.positions);
    NaiveNN3D naiveNN3D(medialAxis.positions);

    for(unsigned int i=0 ; i<osp.surfacePoints.size()-1 ; ++i){
        std::tuple<Eigen::Vector3d, double, double> pM;
        Eigen::Vector3d pA = osp.surfacePoints[i].position;
        Eigen::Vector3d pB;
        if(medialAxis.genMedialPoint(pM, pB, i, i+1)!=0)
            continue;
        std::vector< std::vector< std::pair<Eigen::Vector3d, double> > > outFlann3D, outNaiveNN3D;
        std::vector<Eigen::Vector3d> queries;
        queries.push_back(std::get<0>(pM));
        flann3D.knnSearch(queries, outFlann3D, 2);
        naiveNN3D.knnSearch(queries, outNaiveNN3D, 2);
        CHECK_CLOSE((pA-std::get<0>(pM)).norm(), (pB-std::get<0>(pM)).norm(), 0.000001);
        CHECK_CLOSE((pA-std::get<0>(pM)).norm(), std::get<1>(pM), 0.000001);
        CHECK_CLOSE(outFlann3D[0][0].second, outFlann3D[0][1].second, 0.000001);
        CHECK_CLOSE(outFlann3D[0][0].second, (pA-std::get<0>(pM)).norm(), 0.000001);
        CHECK_CLOSE(outNaiveNN3D[0][0].second, outNaiveNN3D[0][1].second, 0.000001);
        CHECK_CLOSE(outNaiveNN3D[0][0].second, (pA-std::get<0>(pM)).norm(), 0.000001);
    }

}

}
