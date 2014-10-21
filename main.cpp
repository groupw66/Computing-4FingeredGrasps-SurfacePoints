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

        //randomUniform
        printf("start RandomUniform\n");
        std::vector<Eigen::Vector3d> samplePoints = SamplingPoints::randomUniform(osp.minAABB, osp.maxAABB, nSamplePoint);
        printf("SamplingPoints::randomUniform\n");
        //std::vector<std::vector<Grasp> > sol;
        //Compute4FingeredGrasps::compute4FingeredGrasps(sol, osp.surfacePoints, samplePoints, halfAngle);
        std::set<Grasp> solSet;
        std::vector<int> sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, samplePoints, halfAngle);
        printf("Compute4FingeredGrasps success\n");

        std::ofstream ofsUniform ("out/"+fName + ".uniform.out", std::ofstream::ate);
        ofsUniform.unsetf ( std::ios::floatfield );
        ofsUniform.precision(std::numeric_limits<long double>::digits10);
        /*
        for(unsigned int i=0 ; i < samplePoints.size() ; ++i){
            ofsUniform << samplePoints[i].x() << " " << samplePoints[i].y() << " " << samplePoints[i].z() << "\n";
            ofsUniform << sol[i].size() << "\n";
            for(Grasp g : sol[i]){
                ofsUniform << g.surfacePoints.size() << "\n";
                for(SurfacePoint sp : g.surfacePoints){
                    ofsUniform << sp.position.x() << " " << sp.position.y() << " " << sp.position.z() << " ";
                    ofsUniform << sp.normal.x() << " " << sp.normal.y() << " " << sp.normal.z() << "\n";
                }
            }
        }
        */
        ofsUniform << sizeSols.size() << "\n";
        for(int sizeSol : sizeSols){
            ofsUniform << sizeSol << "\n";
        }
        ofsUniform.close();
        printf("finish RandomUniform\n");

        //randomNormalDist
        printf("start randomNormalDist\n");
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        samplePoints = SamplingPoints::randomNormalDist(osp.cm, diffAABB/6., nSamplePoint);
        printf("SamplingPoints::randomNormalDist\n");
        //std::vector<std::vector<Grasp> > sol;
        //Compute4FingeredGrasps::compute4FingeredGrasps(sol, osp.surfacePoints, samplePoints, halfAngle);
        sizeSols = Compute4FingeredGrasps::compute4FingeredGrasps(solSet, osp.surfacePoints, samplePoints, halfAngle);
        printf("Compute4FingeredGrasps success\n");

        std::ofstream ofsNormalDist ("out/"+fName + ".normalDist.out", std::ofstream::ate);
        ofsNormalDist.unsetf ( std::ios::floatfield );
        ofsNormalDist.precision(std::numeric_limits<long double>::digits10);
        ofsNormalDist << sizeSols.size() << "\n";
        for(int sizeSol : sizeSols){
            ofsNormalDist << sizeSol << "\n";
        }
        ofsNormalDist.close();
        printf("finish randomNormalDist\n");

    }
    else{
        return UnitTest::RunAllTests();
    }
    return 0;
}
