#include <UnitTest++.h>
#include <Eigen/Dense>
#include "ObjectSurfacePoints.h"

namespace
{

TEST(ObjectSurfacePoints_ctor_OBJFile_Amicelli_800)
{
    OBJFile obj("test/meshes/KIT/Amicelli_800.obj");
    //int nPoints = 400;
    int nFacets = 798;
    ObjectSurfacePoints osp(obj);
    CHECK_EQUAL(nFacets, osp.surfacePoints.size());
}

TEST(ObjectSurfacePoints_ctor_PositionsNormalsFile_ammo50)
{
    PositionsNormalsFile obj("test/meshes/spectralMesh/ammo50.txt");
    int nPoints = 50;
    ObjectSurfacePoints osp(obj);
    CHECK_EQUAL(nPoints, osp.surfacePoints.size());
}

}
