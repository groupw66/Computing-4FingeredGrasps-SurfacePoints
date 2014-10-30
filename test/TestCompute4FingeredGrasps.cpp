#include <UnitTest++.h>
#include <Eigen/Dense>
#include <cmath>
#include "Compute4FingeredGrasps.h"
#include "PositionsNormalsFile.h"
#include "OBJFile.h"
#include "ObjectSurfacePoints.h"
#include "SamplingPoints.h"
#include "Timer.h"

namespace
{

TEST(findEquilibriumGrasps_forceDual_solsSet)
{
    std::string objFilename = "test/meshes/KIT/CatLying_800.obj";
    double timelimit = 500, halfAngle = 10.d;
    ObjectSurfacePoints osp;
    OBJFile obj(objFilename.c_str());
    fflush(stdout);
    osp.open(obj);
    std::vector<std::tuple<double, Grasp, double> > sols, sols2;
    std::unordered_set<std::string> solsSet, solsSet2;
    int nTest = 0;
    Timer tmr;
    std::random_device rd;
    std::default_random_engine rng(rd());
    Eigen::Vector3d minAABB = osp.minAABB,
                    maxAABB = osp.maxAABB;
    std::uniform_real_distribution<double> randUX(minAABB.x(), maxAABB.x());
    std::uniform_real_distribution<double> randUY(minAABB.y(), maxAABB.y());
    std::uniform_real_distribution<double> randUZ(minAABB.z(), maxAABB.z());
    //std::unordered_set<std::tuple<double,double,double> > uniquePoints;
    tmr.reset();
    while(std::min(sols.size(), sols2.size()) < 500){
        printf("- %lf \n",tmr.elapsed());
        Eigen::Vector3d sampledPoint = Eigen::Vector3d(randUX(rng), randUY(rng), randUZ(rng));
        nTest++;
        std::vector<unsigned int> filteredPointIds;
        Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, sampledPoint, halfAngle);

        std::vector<Grasp> fcGrasps;
        if(filteredPointIds.size() >= 4){
            Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                        fcGrasps, filteredPointIds, sampledPoint, osp.surfacePoints);
            Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                        sols2, solsSet2, tmr, timelimit, halfAngle,
                                        filteredPointIds, sampledPoint, osp.surfacePoints);
        }
        for(Grasp g : fcGrasps){
            if(tmr.elapsed() >= timelimit)
                break;
            if(!solsSet.insert(g.to_str()).second)
                continue;
            double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[g[0]], osp.surfacePoints[g[1]],
                                                     osp.surfacePoints[g[2]], osp.surfacePoints[g[3]],
                                                     Eigen::Vector3d(0,0,0), halfAngle);
            sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
        }
    }
    printf("size: %d %d\n",sols.size(), sols2.size() );
    CHECK_EQUAL(sols.size(), sols2.size());
    for(int i = 0 ; i < sols.size() ; ++i){
        CHECK_CLOSE(std::get<0>(sols[i]), std::get<0>(sols2[i]),1e-6);
    }
}

inline void test_uniformAxis(ObjectSurfacePoints &osp, int npointsPerAxis, double halfAngle, int nUniqueSol)
{
    Timer tmr;

    std::vector<Eigen::Vector3d> samplePoints = SamplingPoints::uniformAxis(osp.minAABB, osp.maxAABB, npointsPerAxis);
    /*
    std::vector<std::vector<Grasp> > sol;
    tmr.reset();
    Compute4FingeredGrasps::compute4FingeredGrasps(sol, osp.surfacePoints, samplePoints, halfAngle);
    std::cout << "compute4FingeredGrasps: " << tmr.elapsed() << " s" << std::endl;
    std::vector<Grasp> uniqueSol;
    Compute4FingeredGrasps::uniqueSol(uniqueSol, sol);
    CHECK_EQUAL(nUniqueSol, uniqueSol.size());
    */
    /*
    std::vector<std::vector<Grasp> > sol_naive;
    tmr.reset();
    Compute4FingeredGrasps::compute4FingeredGrasps_naive(sol_naive, osp.surfacePoints, samplePoints, halfAngle);
    std::cout << "compute4FingeredGrasps_naive: " << tmr.elapsed() << " s" << std::endl;
    std::vector<Grasp> uniqueSol_naive;
    Compute4FingeredGrasps::uniqueSol(uniqueSol_naive, sol_naive);
    CHECK_EQUAL(nUniqueSol, uniqueSol_naive.size());
    */
    std::set<Grasp> solSet;
    tmr.reset();
    std::vector<int> sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, samplePoints, halfAngle);
    std::cout << "compute4FingeredGrasps set: " << tmr.elapsed() << " s" << std::endl;
    CHECK_EQUAL(nUniqueSol, solSet.size());
}

TEST(compute4FingeredGrasps_cow500_16PointsPerAxis)
{
    char filename[1000] = "test/meshes/spectralMesh/cow500.txt";
    int npointsPerAxis = 16;
    double halfAngle = 10.d;
    int nUniqueSol = 35255;

    PositionsNormalsFile obj(filename);
    ObjectSurfacePoints osp(obj);
    test_uniformAxis(osp, npointsPerAxis, halfAngle, nUniqueSol);
}

TEST(compute4FingeredGrasps_CatLying_800_8PointsPerAxis)
{
    char filename[1000] = "test/meshes/KIT/CatLying_800.obj";
    int npointsPerAxis = 8;
    double halfAngle = 10.d;
    int nUniqueSol = 126939;

    OBJFile obj(filename);
    ObjectSurfacePoints osp(obj);
    test_uniformAxis(osp, npointsPerAxis, halfAngle, nUniqueSol);
}


//isPointInCone

TEST(isPointInCone_true0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1,-1,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInCone_true1)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.000005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInCone_false0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.500005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(false, Result);
}

TEST(isPointInConeDoubleside_true0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.000005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInConeDoubleside_true1)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(0,1,0);
    Eigen::Vector3d normal(-1.000005,1.00005,0);
    Eigen::Vector3d position(1,0,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInConeDoubleside_false0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.500005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(false, Result);
}



}
