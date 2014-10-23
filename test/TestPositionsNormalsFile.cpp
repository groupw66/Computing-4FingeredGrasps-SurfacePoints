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

TEST(PositionsNormalsFile_write)
{
    PositionsNormalsFile obj("test/meshes/spectralMesh/ammo50.txt");
    PositionsNormalsFile out(obj.positions, obj.normals);
    out.write("test/tmp.txt");
    PositionsNormalsFile readOut("test/tmp.txt");
    CHECK_EQUAL(readOut.positions.size(), obj.positions.size());
    CHECK_EQUAL(readOut.normals.size(), obj.normals.size());
    CHECK_ARRAY_EQUAL(obj.positions, readOut.positions, obj.positions.size());
    CHECK_ARRAY_EQUAL(obj.normals, readOut.normals, obj.normals.size());
    std::remove("test/tmp.txt");
}

}
