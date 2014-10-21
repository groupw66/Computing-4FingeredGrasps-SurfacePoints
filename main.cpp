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

int main(int argc,char *argv[])
{
    if(argc > 1){
        std::string filename;
        int nSamplePoint = 10000;
        double halfAngle = 10.d;
        if(argc >= 2){
            filename = argv[1];
        }
        if(argc >= 3){
            nSamplePoint = atoi(argv[2]);
        }
        if(argc >= 4){
            halfAngle = atof(argv[3]);
        }
        //open object model file
        ObjectSurfacePoints osp;
        std::string fExt = filename.substr(filename.find_last_of(".") + 1);
        std::string fName = filename.substr(filename.find_last_of("/") + 1);
        if(fExt == "obj") {
            OBJFile obj(filename.c_str());
            osp.open(obj);
        }
        else if(fExt == "txt") {
            PositionsNormalsFile obj(filename.c_str());
            osp.open(obj);
        }
        else {
            std::cout << "Not supported file..." << std::endl;
        }

        printf("\n%s\n", filename.c_str());
        std::vector<Eigen::Vector3d> sampledPoints;
        std::set<Grasp> solSet;
        std::vector<int> sizeSols;
        std::ofstream ofs;
        ofs.unsetf ( std::ios::floatfield );
        ofs.precision(std::numeric_limits<long double>::digits10);

        // SamplingPoints::randomUniform
        printf("SamplingPoints::randomUniform start\n");
        sampledPoints = SamplingPoints::randomUniform(osp.minAABB, osp.maxAABB, nSamplePoint);
        printf("SamplingPoints::randomUniform ok\n");
        /*
        printf("Write .uniform.sampledPoints start\n");
        ofs.open(fName + ".uniform.sampledPoints", std::ofstream::ate);
        ofs << sampledPoints.size() << "\n";
        for(Eigen::Vector3d sp : sampledPoints){
            ofs << sp.x() << " " << sp.y() << " " << sp.z() << "\n";
        }
        ofs.close();
        printf("Write .uniform.sampledPoints ok\n");
        */
        printf("Compute4FingeredGrasps start\n");
        sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, sampledPoints, halfAngle);
        printf("Compute4FingeredGrasps ok\n");

        printf("Write .uniform.sizeSols start\n");
        ofs.open(fName + ".uniform.sizeSols", std::ofstream::ate);
        ofs << sizeSols.size() << "\n";
        for(int sizeSol : sizeSols){
            ofs << sizeSol << "\n";
        }
        ofs.close();
        printf("Write .uniform.sizeSols ok\n");

        printf("\n");

        //randomNormalDist
        printf("SamplingPoints::randomNormalDist start\n");
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        sampledPoints = SamplingPoints::randomNormalDist(osp.cm, diffAABB/6., nSamplePoint);
        printf("SamplingPoints::randomNormalDist ok\n");
        /*
        printf("Write .normalDist.sampledPoints start\n");
        ofs.open(fName + ".normalDist.sampledPoints", std::ofstream::ate);
        ofs << sampledPoints.size() << "\n";
        for(Eigen::Vector3d sp : sampledPoints){
            ofs << sp.x() << " " << sp.y() << " " << sp.z() << "\n";
        }
        ofs.close();
        printf("Write .normalDist.sampledPoints ok\n");
        */
        printf("Compute4FingeredGrasps start\n");
        sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, sampledPoints, halfAngle);
        printf("Compute4FingeredGrasps ok\n");

        printf("Write .normalDist.sizeSols start\n");
        ofs.open(fName + ".normalDist.sizeSols", std::ofstream::ate);
        ofs << sizeSols.size() << "\n";
        for(int sizeSol : sizeSols){
            ofs << sizeSol << "\n";
        }
        ofs.close();
        printf("Write .normalDist.sizeSols ok\n");

        printf("--------------------------------\n");
    }
    else{
        return UnitTest::RunAllTests();
    }
    return 0;
}
