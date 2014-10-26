#include <iostream>
#include <fstream>
#include <UnitTest++.h>
#include <cstring>
#include <map>
#include "SamplingPoints.h"
#include "Compute4FingeredGrasps.h"
#include "OBJFile.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"
#include "ForceClosure.h"
#include "Timer.h"
#include "dae/DaeHeuristic.h"

using namespace std;

void runCompute4FingeredGrasps(std::string randomMode, std::string objFilename, std::string outFilename, int nSamplePoint, double halfAngle)
{
    std::ifstream objFile(objFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "!" << objFilename << std::endl;
        return;
    }
    objFile.close();
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
    else if(randomMode == "step"){
        Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
        sampledPoints = SamplingPoints::uniform(osp.minAABB, osp.maxAABB, nSamplePoint);
    }
    sampleRuntime = tmr.elapsed();
    ofs.open(outFilename + ".sampledPoints", std::ofstream::ate);
    ofs << sampledPoints.size() << "\n";
    for(Eigen::Vector3d sp : sampledPoints){
        ofs << sp.x() << " " << sp.y() << " " << sp.z() << "\n";
    }
    ofs.close();

    // Compute4FingeredGrasps
    printf("%s | Compute4FingeredGrasps start\n", outFilename.c_str());
    ofs.open(outFilename + ".sols", std::ofstream::ate);
    ofs << sampledPoints.size() << "\n";
    std::set<Grasp> solSet;
    std::vector<int> sizeSols;
    std::vector<std::vector<Grasp> > sols;
    sols.resize(sampledPoints.size());

    tmr.reset();
    //#pragma omp parallel for schedule(static, 1)
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
            /*
            for(Grasp g : fcGrasps){
                if(solSet.insert(g).second){
                    //sols[i].push_back(g);
                }
            }
            sols[i].insert(sols[i].end(), fcGrasps.begin(), fcGrasps.end());
            sizeSols.push_back(solSet.size());
            */
            ofs << fcGrasps.size() << "\n";
            for(Grasp g : fcGrasps){
                for(unsigned int id : g.surfacePoints){
                    ofs << id << " ";
                }
                ofs << "\n";
            }

        }
        eachSampleRuntimes.push_back(tmr.elapsed());
    }
    printf("%s | Compute4FingeredGrasps ok\n", outFilename.c_str());
    ofs.close();

    ofs.open(outFilename + ".runtime", std::ofstream::ate);
    ofs << "sampleRuntime: " << sampleRuntime << "\n";
    ofs << "pointInConesRuntimes: " << pointInConesRuntimes << "\n";
    ofs << "findEquilibriumRuntimes: " << findEquilibriumRuntimes << "\n";
    ofs << "eachSampleRuntimes\n" << eachSampleRuntimes.size() << "\n";
    for(double eachSampleRuntime : eachSampleRuntimes){
        ofs << eachSampleRuntime << "\n";
    }
    ofs.close();
    /*
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
    */
    printf("%s | Write output ok\n", outFilename.c_str());
    printf("---------------------------------------\n");
}

void cvtOBJtoSurfacePoints(std::string objFilename, std::string outFilename)
{
    std::ifstream objFile(objFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "!" << objFilename << std::endl;
        return;
    }
    objFile.close();
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
    std::ifstream objFile(surfacePointFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "!" << surfacePointFilename << std::endl;
        return;
    }
    objFile.close();
    PositionsNormalsFile obj(surfacePointFilename.c_str());
    ObjectSurfacePoints osp(obj);
    std::ifstream solFile(solFilename);
    std::ofstream outFile(outFilename);
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!solFile.is_open()){
        std::cout << "!" << solFilename << std::endl;
        return;
    }
    if(!outFile.is_open()){
        std::cout << "!" << outFilename << std::endl;
        return;
    }
    int n;
    solFile >> n;
    outFile << n << "\n";
    Timer tmr;
    for(int i=0 ; i<n; ++i){
        int nSol;
        solFile >> nSol;
        outFile << nSol << "\n";
        tmr.reset();
        for(int j=0 ; j<nSol ; ++j){
            int a, b, c, d;
            solFile >> a >> b >> c >> d;
            double mindist = ForceClosure::getMindist_Qhull(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
            outFile << mindist << "\n";
            //std::cout << mindist << std::endl;
        }
        if(nSol>0)
            std::cout << nSol << " : " << tmr.elapsed()/nSol << std::endl;
    }
    solFile.close();
    outFile.close();

}

void allFC(std::string surfacePointFilename, std::string outFilename, double halfAngle = 10.d)
{
    Timer tmr;
    std::ifstream objFile(surfacePointFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "!" << surfacePointFilename << std::endl;
        return;
    }
    objFile.close();
    PositionsNormalsFile obj(surfacePointFilename.c_str());
    ObjectSurfacePoints osp(obj);
    std::ofstream outFile(outFilename);
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "!" << outFilename << std::endl;
        return;
    }
    DaeHeuristicChecker daeHeuristicChecker(halfAngle * M_PI / 180.d);
    unsigned int nSurfacePoint = osp.surfacePoints.size();
    for(unsigned int a=0 ; a<nSurfacePoint ; ++a){
        std::cout << outFilename << " : a = " << a << std::endl;
        for(unsigned int b=a+1 ; b<nSurfacePoint ; ++b){
            for(unsigned int c=b+1 ; c<nSurfacePoint ; ++c){
                for(unsigned int d=c+1 ; d<nSurfacePoint ; ++d){
                    //std::cout << a << " " << b << " " << c << " " << d << std::endl;
                    bool passFilter = daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d]);
                    if(passFilter){
                        double mindist = ForceClosure::getMindist_Qhull(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
                        if(mindist > 0){
                            outFile << a << " " << b << " " << c << " " << d << " " << mindist << "\n";
                        }
                    }
                }
            }
        }
    }
}

void allFC_allFCSorted(std::string allFCFilename, std::string outFilename)
{
    std::ifstream allFCFile(allFCFilename.c_str());
    if(!allFCFile.is_open()){
        std::cout << "! Can't open " << allFCFilename << std::endl;
        return;
    }
    std::ofstream outFile(outFilename.c_str());
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    std::vector<pair<double, Grasp> > allFCs;
    std::string line;
    while (std::getline(allFCFile, line))
    {
        std::istringstream iss(line);
        int a, b, c, d;
        double mindist;
        if (!(iss >> a >> b >> c >> d >> mindist)) { //error
            std::cout << "!(iss >> a >> b >> c >> d >> mindist) : " << allFCs.size()+1 << std::endl;
            break;
        }
        allFCs.push_back(std::make_pair(mindist, Grasp(a, b, c, d)));
    }
    std::sort(allFCs.begin(), allFCs.end(), std::greater<pair<double, Grasp> >());
    for(unsigned int i=0 ; i<allFCs.size() ; ++i){
        outFile << allFCs[i].second[0] << " " <<
                    allFCs[i].second[1] << " " <<
                    allFCs[i].second[2] << " " <<
                    allFCs[i].second[3] << " " <<
                    allFCs[i].first << "\n" ;
    }
    outFile.close();
    allFCFile.close();
}


void sols_solsMindist(std::string allFCFilename, std::string solsFilename, std::string outFilename)
{
    std::ifstream allFCFile(allFCFilename.c_str());
    if(!allFCFile.is_open()){
        std::cout << "! Can't open " << allFCFilename << std::endl;
        return;
    }
    std::ifstream solsFile(solsFilename.c_str());
    if(!solsFile.is_open()){
        std::cout << "! Can't open " << solsFilename << std::endl;
        return;
    }
    std::ofstream outFile(outFilename.c_str());
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    std::map<Grasp, double> allFCmap;
    std::string line;
    while (std::getline(allFCFile, line))
    {
        std::istringstream iss(line);
        int a, b, c, d;
        double mindist;
        if (!(iss >> a >> b >> c >> d >> mindist)) { //error
            std::cout << "!(iss >> a >> b >> c >> d >> mindist) : " << allFCmap.size()+1 << std::endl;
            break;
        }
        allFCmap.insert(allFCmap.end(),std::make_pair(Grasp(a, b, c, d), mindist));
    }
    int n;
    solsFile >> n;
    outFile << n << "\n";
    for(int i=0 ; i<n; ++i){
        int nSol;
        solsFile >> nSol;
        outFile << nSol << "\n";
        for(int j=0 ; j<nSol ; ++j){
            int a, b, c, d;
            solsFile >> a >> b >> c >> d;
            auto it = allFCmap.find(Grasp(a, b, c, d));
            if(it != allFCmap.end()){
                outFile << a << " " << b << " " << c << " " << d << " " << it->second << "\n";
            }
            else{
                std::cout << "Grasp " << a << " " << b << " " << c << " " << d << "not found!" << std::endl;
            }
        }
    }
    allFCFile.close();
    solsFile.close();
    outFile.close();
}

int main(int argc,char *argv[])
{
    if(argc > 1){
        std::string mode;
        if(argc >= 2){
            mode = argv[1];
            if(mode == "uniform" || mode == "normalDist" || mode == "normalDistLimit" || mode == "step"){
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
                cvtOBJtoSurfacePoints(argv[2], //objFilename
                                      argv[3] //outFilenmae
                                      );
            }
            else if(mode == "mindist"){
                computeMindist(argv[2], //surfacePointsFilename
                               argv[3], //solsFilename
                               argv[4], //outFilename
                               atof(argv[5]) //halfAngle(degree)
                               );
            }
            else if(mode == "allFC"){
                allFC(argv[2], //surfacePointsFilename
                      argv[3], //outFilename
                      atof(argv[4]) //halfAngle(degree)
                      );
            }
            else if(mode == "allFC_allFCSorted"){
                allFC_allFCSorted(argv[2], //allFCFilename
                                  argv[3] //outFilename
                                  );
            }
            else if(mode == "sols_solsMindist"){
                sols_solsMindist(argv[2], //allFCFilename
                                  argv[3], //solsFilename
                                  argv[4] //outFilename
                                  );
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
