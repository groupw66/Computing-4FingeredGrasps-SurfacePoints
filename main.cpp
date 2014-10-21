#include <iostream>
#include <fstream>
#include <UnitTest++.h>
#include <cstring>
#include "SamplingPoints.h"
#include "Compute4FingeredGrasps.h"
#include "OBJFile.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"

using namespace std;

void runCompute4FingeredGrasps(std::string randomMode, std::string objFilename, std::string outFilename, int nSamplePoint, double halfAngle)
{
    std::cout << outFilename << std::endl;
    ObjectSurfacePoints osp;
    std::vector<Eigen::Vector3d> sampledPoints;
    std::set<Grasp> solSet;
    std::vector<int> sizeSols;
    std::ofstream ofs;
    ofs.unsetf ( std::ios::floatfield );
    ofs.precision(std::numeric_limits<long double>::digits10);

    //open object model file
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

    // SamplingPoints
    if(randomMode == "-ru"){
        outFilename;
        printf("SamplingPoints::randomUniform start\n");
        sampledPoints = SamplingPoints::randomUniform(osp.minAABB, osp.maxAABB, nSamplePoint);
        printf("SamplingPoints::randomUniform ok\n");
    }
    else if(randomMode == "-rn"){
        printf("SamplingPoints::randomNormalDist start\n");
        sampledPoints = SamplingPoints::randomNormalDist(osp.minAABB, osp.maxAABB, nSamplePoint);
        printf("SamplingPoints::randomNormalDist ok\n");
    }

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
    sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, sampledPoints, halfAngle);
    printf("Compute4FingeredGrasps ok\n");

    printf("Write .sizeSols start\n");
    ofs.open(outFilename + ".sizeSols", std::ofstream::ate);
    ofs << sizeSols.size() << "\n";
    for(int sizeSol : sizeSols){
        ofs << sizeSol << "\n";
    }
    ofs.close();
    printf("Write .sizeSols ok\n");

    printf("--------------------\n");
}

int main(int argc,char *argv[])
{
    if(argc > 1){
        std::string mode;
        if(argc >= 2){
            mode = argv[1];
            if(mode == "-ru" || mode == "-rn"){
                std::string objFilename;
                std::string outFilename;
                int nSamplePoint = 10000;
                double halfAngle = 10.d;
                if(argc >= 3){
                    objFilename = argv[2];
                    outFilename = objFilename.substr(objFilename.find_last_of("/") + 1);
                    if(mode == "-ru")
                        outFilename += ".uniform";
                    else if(mode == "-rn")
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
