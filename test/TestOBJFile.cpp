#include <UnitTest++.h>
#include <Eigen/Dense>
#include "OBJFile.h"

namespace
{

TEST(OBJFile_Amicelli_800)
{
    OBJFile obj("meshes/KIT/Amicelli_800.obj");
    int nPoints = 400;
    int nFacets = 798;
    CHECK_EQUAL(nPoints, obj.vertexs.size());
    CHECK_EQUAL(nFacets, obj.facets.size());
}

}
