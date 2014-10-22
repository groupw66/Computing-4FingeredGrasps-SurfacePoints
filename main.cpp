#include <iostream>
#include <fstream>
#include <UnitTest++.h>
#include <cstring>
#include "SamplingPoints.h"
#include "Compute4FingeredGrasps.h"
#include "OBJFile.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"
#include <chrono>

using namespace std;

//https://gist.github.com/gongzhitaao/7062087
class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

void runCompute4FingeredGrasps(std::string randomMode, std::string objFilename, std::string outFilename, int nSamplePoint, double halfAngle)
{
    std::cout << outFilename << std::endl;

    // runtime
    Timer tmr;
    Timer tmr2;
    double sampleRuntime = 0.d;
    double pointInConesRuntimes = 0.d;
    double findEquilibriumRuntimes = 0.d;
    std::vector<double> eachSampleRuntimes;

    std::ofstream ofs;
    ofs.unsetf ( std::ios::floatfield );
    ofs.precision(std::numeric_limits<long double>::digits10);

    //open object model file
    ObjectSurfacePoints osp;
    std::string objFileExt = objFilename.substr(objFilename.find_last_of(".") + 1);
    if(objFileExt == "obj") {
        OBJFile obj(objFilename.c_str());
        osp.open(obj);
    }
    else if(objFileExt == "txt") {
        PositionsNormalsFile obj(objFilename.c_str());
        osp.open(obj);
    }
    else {
        std::cout << "Not supported file..." << std::endl;
    }
    printf("Write .surfacePoints start\n");
    ofs.open(outFilename + ".surfacePoints", std::ofstream::ate);
    ofs << osp.surfacePoints.size() << "\n";
    for(SurfacePoint sp : osp.surfacePoints){
        ofs << sp.position.x() << " " << sp.position.y() << " " << sp.position.z() << " ";
        ofs << sp.normal.x() << " " << sp.normal.y() << " " << sp.normal.z() << "\n";
    }
    ofs.close();
    printf("Write .surfacePoints ok\n");


    // SamplingPoints
    std::vector<Eigen::Vector3d> sampledPoints;
    tmr.reset();
    if(randomMode == "uniform"){
        printf("SamplingPoints::randomUniform start\n");
        sampledPoints = SamplingPoints::randomUniform(osp.minAABB, osp.maxAABB, nSamplePoint);
        printf("SamplingPoints::randomUniform ok\n");
    }
    else if(randomMode == "normalDist"){
        printf("SamplingPoints::randomNormalDist start\n");
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        sampledPoints = SamplingPoints::randomNormalDist(osp.cm, diffAABB/6., nSamplePoint);
        printf("SamplingPoints::randomNormalDist ok\n");
    }
    sampleRuntime = tmr.elapsed();

    printf("Write .sampledPoints start\n");
    ofs.open(outFilename + ".sampledPoints", std::ofstream::ate);
    ofs << sampledPoints.size() << "\n";
    for(Eigen::Vector3d sp : sampledPoints){
        ofs << sp.x() << " " << sp.y() << " " << sp.z() << "\n";
    }
    ofs.close();
    printf("Write .sampledPoints ok\n");

    // Compute4FingeredGrasps
    printf("Compute4FingeredGrasps start\n");
    std::set<Grasp> solSet;
    std::vector<int> sizeSols;
    std::vector<std::vector<Grasp> > sols;
    sols.resize(sampledPoints.size());
    //sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, sampledPoints, halfAngle);
    //#pragma omp parallel for schedule(static, 1)
    tmr.reset();
    for(unsigned int i=0 ; i<sampledPoints.size() ; ++i){
        tmr2.reset();
        std::vector<unsigned int> filteredPointIds;
        Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, sampledPoints[i], halfAngle);
        pointInConesRuntimes += tmr2.elapsed();

        tmr2.reset();
        std::vector<Grasp> fcGrasps;
        if(filteredPointIds.size() >= 4){
            Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(fcGrasps, filteredPointIds, sampledPoints[i], osp.surfacePoints);
        }
        findEquilibriumRuntimes += tmr2.elapsed();
        //#pragma omp critical
        {
            //solSet.insert(fcGrasps.begin(),fcGrasps.end());
            for(Grasp g : fcGrasps){
                if(solSet.insert(g).second){
                    sols[i].push_back(g);
                }
            }
            sizeSols.push_back(solSet.size());
        }
        eachSampleRuntimes.push_back(tmr.elapsed());
    }
    printf("Compute4FingeredGrasps ok\n");

    printf("Write .runtime start\n");
    ofs.open(outFilename + ".runtime", std::ofstream::ate);
    ofs << "sampleRuntime: " << sampleRuntime << "\n";
    ofs << "pointInConesRuntimes: " << pointInConesRuntimes << "\n";
    ofs << "findEquilibriumRuntimes: " << findEquilibriumRuntimes << "\n";
    ofs << "eachSampleRuntimes\n" << eachSampleRuntimes.size() << "\n";
    for(double eachSampleRuntime : eachSampleRuntimes){
        ofs << eachSampleRuntime << "\n";
    }
    ofs.close();
    printf("Write .runtime ok\n");

    printf("Write .sizeSols start\n");
    ofs.open(outFilename + ".sizeSols", std::ofstream::ate);
    ofs << sizeSols.size() << "\n";
    for(int sizeSol : sizeSols){
        ofs << sizeSol << "\n";
    }
    ofs.close();
    printf("Write .sizeSols ok\n");

    printf("Write .sols start\n");
    ofs.open(outFilename + ".sols", std::ofstream::ate);
    ofs << sols.size() << "\n";
    for(std::vector<Grasp> sol : sols){
        ofs << sol.size() << "\n";
        for(Grasp g : sol){
            for(unsigned int id : g.surfacePoints){
                ofs << id << " ";
            }
            ofs << "\n";
        }
    }
    ofs.close();
    printf("Write .sols ok\n");

    printf("--------------------\n");
}

int main(int argc,char *argv[])
{
    // (-ru,-rn) objFilename outFilename nSamplePoint halfAngle
    if(argc > 1){
        std::string mode;
        if(argc >= 2){
            mode = argv[1];
            if(mode == "uniform" || mode == "normalDist"){
                std::string objFilename;
                std::string outFilename;
                int nSamplePoint = 10000;
                double halfAngle = 10.d;
                if(argc >= 3){
                    objFilename = argv[2];
                    outFilename = objFilename.substr(objFilename.find_last_of("/") + 1);
                    if(mode == "uniform")
                        outFilename += ".uniform";
                    else if(mode == "normalDist")
                        outFilename += ".normalDist";
                }
                if(argc >= 4){
                    outFilename = argv[3];
                }
                if(argc >= 5){
                    nSamplePoint = atoi(argv[4]);
                }
                if(argc >= 6){
                    halfAngle = atof(argv[5]);
                }
                runCompute4FingeredGrasps(mode, objFilename, outFilename, nSamplePoint, halfAngle);
            }
            else{
                std::cout << "Unknown command..." << std::endl;
            }
        }
    }
    else{
        return UnitTest::RunAllTests();
    }
    return 0;
}
