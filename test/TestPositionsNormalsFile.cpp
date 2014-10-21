#include <UnitTest++.h>
#include <Eigen/Dense>
#include "PositionsNormalsFile.h"

namespace
{

TEST(PositionsNormalsFile_ammo50)
{
    PositionsNormalsFile obj("test/meshes/spectralMesh/ammo50.txt");
    int nPoints = 50;
    CHECK_EQUAL(nPoints, obj.positions.size());
    CHECK_EQUAL(nPoints, obj.normals.size());
    CHECK_EQUAL(obj.positions.size(), obj.normals.size());
}

}
