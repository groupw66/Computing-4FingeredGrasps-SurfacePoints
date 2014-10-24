#include <iostream>
#include <fstream>
#include <UnitTest++.h>
#include <cstring>
#include "SamplingPoints.h"
#include "Compute4FingeredGrasps.h"
#include "OBJFile.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"
#include "ForceClosure.h"
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

    // SamplingPoints
    std::vector<Eigen::Vector3d> sampledPoints;
    tmr.reset();
    if(randomMode == "uniform"){
        sampledPoints = SamplingPoints::randomUniform(osp.minAABB, osp.maxAABB, nSamplePoint);
    }
    else if(randomMode == "normalDist"){
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        sampledPoints = SamplingPoints::randomNormalDist(osp.cm, diffAABB/6., nSamplePoint);
    }
    else if(randomMode == "normalDistLimit"){
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        sampledPoints = SamplingPoints::randomNormalDist(osp.cm, diffAABB/6., nSamplePoint, osp.minAABB, osp.maxAABB);
    }
    sampleRuntime = tmr.elapsed();
    /*
    ofs.open(outFilename + ".sampledPoints", std::ofstream::ate);
    ofs << sampledPoints.size() << "\n";
    for(Eigen::Vector3d sp : sampledPoints){
        ofs << sp.x() << " " << sp.y() << " " << sp.z() << "\n";
    }
    ofs.close();
    */

    // Compute4FingeredGrasps
    printf("%s | Compute4FingeredGrasps start\n", outFilename.c_str());
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
    printf("%s | Compute4FingeredGrasps ok\n", outFilename.c_str());

    ofs.open(outFilename + ".runtime", std::ofstream::ate);
    ofs << "sampleRuntime: " << sampleRuntime << "\n";
    ofs << "pointInConesRuntimes: " << pointInConesRuntimes << "\n";
    ofs << "findEquilibriumRuntimes: " << findEquilibriumRuntimes << "\n";
    ofs << "eachSampleRuntimes\n" << eachSampleRuntimes.size() << "\n";
    for(double eachSampleRuntime : eachSampleRuntimes){
        ofs << eachSampleRuntime << "\n";
    }
    ofs.close();

    ofs.open(outFilename + ".sizeSols", std::ofstream::ate);
    ofs << sizeSols.size() << "\n";
    for(int sizeSol : sizeSols){
        ofs << sizeSol << "\n";
    }
    ofs.close();

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
    printf("%s | Write output ok\n", outFilename.c_str());
    printf("---------------------------------------\n");
}

void cvtOBJtoSurfacePoints(std::string objFilename, std::string outFilename)
{
    OBJFile obj(objFilename.c_str());
    ObjectSurfacePoints osp(obj);
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> normals;
    for(SurfacePoint sp : osp.surfacePoints){
        positions.push_back(sp.position);
        normals.push_back(sp.normal);
    }
    PositionsNormalsFile out(positions, normals);
    out.write(outFilename.c_str());
}
void computeMindist(std::string surfacePointFilename, std::string solFilename, std::string outFilename,
                    double halfAngle = 10.d)
{
    PositionsNormalsFile obj(surfacePointFilename.c_str());
    ObjectSurfacePoints osp(obj);
    std::ifstream solFile(solFilename);
    if(!solFile.is_open()){
        std::cout << "!" << solFilename << std::endl;
    }
    int n;
    solFile >> n;
    Timer tmr;
    for(int i=0 ; i<n; ++i){
        int nSol;
        solFile >> nSol;
        tmr.reset();
        for(int j=0 ; j<nSol ; ++j){
            int a, b, c, d;
            solFile >> a >> b >> c >> d;
            double mindist = ForceClosure::getMindist_Qhull(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
            //std::cout << mindist << std::endl;
        }
        if(nSol>0)
            std::cout << nSol << " : " << tmr.elapsed()/nSol << std::endl;
    }

}
int main(int argc,char *argv[])
{
    if(argc > 1){
        std::string mode;
        if(argc >= 2){
            mode = argv[1];
            if(mode == "uniform" || mode == "normalDist" || mode == "normalDistLimit"){
                std::string objFilename;
                std::string outFilename;
                int nSamplePoint = 10000;
                double halfAngle = 10.d;
                if(argc >= 3){
                    objFilename = argv[2];
                    outFilename = objFilename.substr(objFilename.find_last_of("/") + 1);
                    outFilename += "." + mode;
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
            else if(mode == "cvtOBJtoSurfacePoints"){
                cvtOBJtoSurfacePoints(argv[2], argv[3]);
            }
            else if(mode == "mindist"){
                computeMindist(argv[2], argv[3], argv[4], atof(argv[5]));
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
