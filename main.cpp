#include <iostream>
#include <fstream>
#include <UnitTest++.h>
#include <cstring>
#include <map>
#include <unordered_set>
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
    std::ifstream testFile(objFilename.c_str());
    if(!testFile.is_open()){
        std::cout << "! Can't open " << objFilename << std::endl;
        return;
    }
    testFile.close();
    std::ofstream testoFile((outFilename+".sols").c_str());
    if(!testoFile.is_open()){
        std::cout << "! Can't open " << (outFilename+".sols") << std::endl;
        return;
    }
    testoFile.close();
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
        std::cout << "Not support object file : " << objFilename << std::endl;
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
        ofs << fcGrasps.size() << "\n";
        for(Grasp g : fcGrasps){
            for(unsigned int id : g.surfacePoints){
                ofs << id << " ";
            }
            ofs << "\n";
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
    printf("%s | Write output ok\n", outFilename.c_str());
    printf("---------------------------------------\n");
}

void cvtOBJtoSurfacePoints(std::string objFilename, std::string outFilename)
{
    std::ifstream objFile(objFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "! Can't open " << objFilename << std::endl;
        return;
    }
    objFile.close();
    std::ofstream outFile(outFilename.c_str());
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    outFile.close();
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
        std::cout << "! Can't open " << surfacePointFilename << std::endl;
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
        std::cout << "! Can't open " << solFilename << std::endl;
        return;
    }
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
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
        std::cout << "! Can't open " << surfacePointFilename << std::endl;
        return;
    }
    objFile.close();
    std::ofstream outFile(outFilename);
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    PositionsNormalsFile obj(surfacePointFilename.c_str());
    ObjectSurfacePoints osp(obj);
    DaeHeuristicChecker daeHeuristicChecker(halfAngle * M_PI / 180.d);
    unsigned int nSurfacePoint = osp.surfacePoints.size();
    for(unsigned int a=0 ; a<nSurfacePoint ; ++a){
        //std::cout << outFilename << " : a = " << a << std::endl;
        for(unsigned int b=a+1 ; b<nSurfacePoint ; ++b){
            for(unsigned int c=b+1 ; c<nSurfacePoint ; ++c){
                for(unsigned int d=c+1 ; d<nSurfacePoint ; ++d){
                    bool passFilter = daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d]);
                    if(passFilter){
                        double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
                        if(mindist > 0){
                            outFile << a << " " << b << " " << c << " " << d << " " << mindist << "\n";
                        }
                    }
                }
            }
        }
    }
    outFile.close();
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
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
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
                outFile << a << " " << b << " " << c << " " << d << " " << 0 << "\n";
                std::cout << "Grasp " << a << " " << b << " " << c << " " << d << "not found!" << std::endl;
            }
        }
    }
    allFCFile.close();
    solsFile.close();
    outFile.close();
    std::remove(solsFilename.c_str());
}

void solsMindist_solsMindistSet(std::string solsMindistFilename, std::string outFilename, double minMindist=0.d)
{
    std::ifstream solsMindistFile(solsMindistFilename.c_str());
    if(!solsMindistFile.is_open()){
        std::cout << "! Can't open " << solsMindistFilename << std::endl;
        return;
    }
    std::ofstream outFile(outFilename.c_str());
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    std::ofstream outSizeFile((outFilename+".size").c_str());
    outSizeFile.unsetf ( std::ios::floatfield );
    outSizeFile.precision(std::numeric_limits<long double>::digits10);
    if(!outSizeFile.is_open()){
        std::cout << "! Can't open " << (outFilename+".size") << std::endl;
        return;
    }
    std::set<Grasp> solsSet;
    int n;
    solsMindistFile >> n;
    outFile << n << "\n";
    outSizeFile << n << "\n";
    for(int i=0 ; i<n; ++i){
        int nSol;
        solsMindistFile >> nSol;
        std::vector<pair<double, Grasp> > tmpSols;
        for(int j=0 ; j<nSol ; ++j){
            int a, b, c, d;
            double mindist;
            solsMindistFile >> a >> b >> c >> d >> mindist;
            if(mindist < minMindist)
                continue;
            if(solsSet.insert(Grasp(a,b,c,d)).second){
                tmpSols.push_back(std::make_pair(mindist, Grasp(a,b,c,d)));
            }
        }
        std::sort(tmpSols.begin(), tmpSols.end(), std::greater<pair<double, Grasp> >());
        outFile << tmpSols.size() << "\n";
        for(auto pdg : tmpSols){
            outFile << pdg.second[0] << " " << pdg.second[1] << " " << pdg.second[2] << " " << pdg.second[3] << " " << pdg.first << "\n";
        }
        outSizeFile << solsSet.size() << "\n";
    }

    solsMindistFile.close();
    outFile.close();
    outSizeFile.close();
}

void solsMindist_solsMindistSet(std::string solsMindistFilename, std::string outFilename,
                                std::string allFCSortedFilename, double percen=100.d)
{
    std::ifstream allFCSortedFile(allFCSortedFilename.c_str());
    if(!allFCSortedFile.is_open()){
        std::cout << "! Can't open " << allFCSortedFilename << std::endl;
        return;
    }
    std::vector<std::pair<Grasp, double> > allFCSorteds;
    std::string line;
    while (std::getline(allFCSortedFile, line))
    {
        std::istringstream iss(line);
        int a, b, c, d;
        double mindist;
        if (!(iss >> a >> b >> c >> d >> mindist)) { //error
            std::cout << "!(iss >> a >> b >> c >> d >> mindist) : " << allFCSorteds.size()+1 << std::endl;
            break;
        }
        allFCSorteds.push_back(std::make_pair(Grasp(a, b, c, d), mindist));
    }
    allFCSortedFile.close();
    int idPercen = allFCSorteds.size()*(percen/100.d) - 1;
    solsMindist_solsMindistSet(solsMindistFilename,outFilename,allFCSorteds[idPercen].second);
}
void solsMindist_solsMindistSorted(std::string solsMindistFilename, std::string outFilename)
{
    std::ifstream solsMindistFile(solsMindistFilename.c_str());
    if(!solsMindistFile.is_open()){
        std::cout << "! Can't open " << solsMindistFilename << std::endl;
        return;
    }
    std::ofstream outFile(outFilename.c_str());
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
    std::set<std::pair<double,Grasp> > solsSet;
    int n;
    solsMindistFile >> n;
    for(int i=0 ; i<n; ++i){
        int nSol;
        solsMindistFile >> nSol;
        for(int j=0 ; j<nSol ; ++j){
            int a, b, c, d;
            double mindist;
            solsMindistFile >> a >> b >> c >> d >> mindist;
            solsSet.insert(std::make_pair(mindist,Grasp(a,b,c,d)));
        }
    }
    outFile << solsSet.size() << "\n";
    for(auto it=solsSet.rbegin() ; it!=solsSet.rend() ; it++){
        outFile << (*it).second[0] << " " << (*it).second[1] << " " << (*it).second[2] << " " << (*it).second[3] << " " << (*it).first << "\n";
    }
    solsMindistFile.close();
    outFile.close();
}

void runCompute4FingeredGraspsFixtime(std::string submode, std::string objFilename, std::string outFilename,
                                      double halfAngle = 10.d, double timelimit = 0.5d)
{
    std::ifstream objFile(objFilename.c_str());
    if(!objFile.is_open()){
        std::cout << "! Can't open " << objFilename << std::endl;
        return;
    }
    objFile.close();
    std::ofstream outFile(outFilename.c_str());
    outFile.unsetf ( std::ios::floatfield );
    outFile.precision(std::numeric_limits<long double>::digits10);
    if(!outFile.is_open()){
        std::cout << "! Can't open " << outFilename << std::endl;
        return;
    }
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
        std::cout << "Not support object file : " << objFilename << std::endl;
    }

    std::vector<std::tuple<double, Grasp, double> > sols;
    std::unordered_set<std::string> solsSet;
    int nTest = 0;
    Timer tmr;
    std::random_device rd;
    std::default_random_engine rng(rd());
    if(submode == "random4P"){
        std::uniform_int_distribution<int> random_int(0,osp.surfacePoints.size()-1);
        DaeHeuristicChecker daeHeuristicChecker(halfAngle * M_PI / 180.d);
        tmr.reset();
        while(tmr.elapsed() < timelimit){
            int a = random_int(rng),
                b = random_int(rng),
                c = random_int(rng),
                d = random_int(rng);
            Grasp grasp(a,b,c,d);
            if(a==b || a==c || a==d || b==c || b==d || c==d)
                continue;
            nTest++;
            bool passFilter = daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b],
                                                                 osp.surfacePoints[c], osp.surfacePoints[d]);
            if(!passFilter)
                continue;
            bool isFC = ForceClosure::isFC_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                 osp.surfacePoints[c], osp.surfacePoints[d],
                                                 Eigen::Vector3d(0,0,0), halfAngle);
            if(!isFC)
                continue;
            if(!solsSet.insert(grasp.to_str()).second)
                continue;
            double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                         osp.surfacePoints[c], osp.surfacePoints[d],
                                                         Eigen::Vector3d(0,0,0), halfAngle);
            if(mindist > 0){
                sols.push_back(std::make_tuple(mindist, grasp, tmr.elapsed()));
            }
        }
    }
    else if(submode == "random4PQhull"){
        std::uniform_int_distribution<int> random_int(0,osp.surfacePoints.size()-1);
        DaeHeuristicChecker daeHeuristicChecker(halfAngle * M_PI / 180.d);
        tmr.reset();
        while(tmr.elapsed() < timelimit){
            int a = random_int(rng),
                b = random_int(rng),
                c = random_int(rng),
                d = random_int(rng);
            Grasp grasp(a,b,c,d);
            if(a==b || a==c || a==d || b==c || b==d || c==d)
                continue;
            nTest++;
            /*
            bool passFilter = daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b],
                                                                 osp.surfacePoints[c], osp.surfacePoints[d]);
            if(!passFilter)
                continue;
            bool isFC = ForceClosure::isFC_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                 osp.surfacePoints[c], osp.surfacePoints[d],
                                                 Eigen::Vector3d(0,0,0), halfAngle);
            if(!isFC)
                continue;
            if(!solsSet.insert(grasp.to_str()).second)
                continue;
                */
            double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                         osp.surfacePoints[c], osp.surfacePoints[d],
                                                         Eigen::Vector3d(0,0,0), halfAngle);
            if(mindist > 0){
                if(solsSet.insert(grasp.to_str()).second)
                    sols.push_back(std::make_tuple(mindist, grasp, tmr.elapsed()));
            }
        }
    }
    else if(submode == "uniform"){
        Eigen::Vector3d minAABB = osp.minAABB,
                        maxAABB = osp.maxAABB;
        std::uniform_real_distribution<double> randUX(minAABB.x(), maxAABB.x());
        std::uniform_real_distribution<double> randUY(minAABB.y(), maxAABB.y());
        std::uniform_real_distribution<double> randUZ(minAABB.z(), maxAABB.z());
        //std::unordered_set<std::tuple<double,double,double> > uniquePoints;
        tmr.reset();
        while(tmr.elapsed() < timelimit){
            Eigen::Vector3d sampledPoint = Eigen::Vector3d(randUX(rng), randUY(rng), randUZ(rng));
            nTest++;
            std::vector<unsigned int> filteredPointIds;
            Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, sampledPoint, halfAngle);

            std::vector<Grasp> fcGrasps;
            if(filteredPointIds.size() >= 4){
                Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                            sols, solsSet, tmr, timelimit, halfAngle,
                                            filteredPointIds, sampledPoint, osp.surfacePoints);
            }
            /*
            for(Grasp g : fcGrasps){
                if(tmr.elapsed() >= timelimit)
                    break;
                if(!solsSet.insert(g.to_str()).second)
                    continue;
                double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[g[0]], osp.surfacePoints[g[1]],
                                                         osp.surfacePoints[g[2]], osp.surfacePoints[g[3]],
                                                         Eigen::Vector3d(0,0,0), halfAngle);
                sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
            }*/
        }
    }
    else if(submode.find("normalDist") == 0){ // submode start with "normalDist"
        double sdDivider = atof(submode.substr(strlen("normalDist")).c_str());
        Eigen::Vector3d mean = Eigen::Vector3d(0,0,0),
                        sd = (osp.maxAABB - osp.minAABB)/sdDivider;
        std::normal_distribution<double> randNX(mean.x(), sd.x());
        std::normal_distribution<double> randNY(mean.y(), sd.y());
        std::normal_distribution<double> randNZ(mean.z(), sd.z());
        //std::unordered_set<std::tuple<double,double,double> > uniquePoints;
        tmr.reset();
        while(tmr.elapsed() < timelimit){
            Eigen::Vector3d sampledPoint = Eigen::Vector3d(randNX(rng), randNY(rng), randNZ(rng));
            nTest++;
            std::vector<unsigned int> filteredPointIds;
            Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, sampledPoint, halfAngle);

            std::vector<Grasp> fcGrasps;
            if(filteredPointIds.size() >= 4){
                Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                            sols, solsSet, tmr, timelimit, halfAngle,
                                            filteredPointIds, sampledPoint, osp.surfacePoints);
            }
            /*
            for(Grasp g : fcGrasps){
                if(tmr.elapsed() >= timelimit)
                    break;
                if(!solsSet.insert(g.to_str()).second)
                    continue;
                double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[g[0]], osp.surfacePoints[g[1]],
                                                         osp.surfacePoints[g[2]], osp.surfacePoints[g[3]],
                                                         Eigen::Vector3d(0,0,0), halfAngle);
                sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
            }*/
        }
    }
    else{
        std::cout << "sub mode should be random4P, uniform or normalDist" << std::endl;
    }
    //write output
    outFile << sols.size() << " " << nTest << "\n";
    for(auto sol : sols){
        outFile << std::get<0>(sol) << " " << std::get<2>(sol) << " " <<
                    std::get<1>(sol)[0] << " " << std::get<1>(sol)[1] << " " << std::get<1>(sol)[2] << " " << std::get<1>(sol)[3] << "\n";
    }
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
            else if(mode == "solsMindist_solsMindistSet"){
                if(argc > 5){
                    solsMindist_solsMindistSet(argv[2], //solsMindistFilename
                                                argv[3], //outFilename
                                                argv[4], //allFCSortedFilename
                                                atof(argv[5]) //percen
                                                );
                }
                else{
                    solsMindist_solsMindistSet(argv[2], //solsMindistFilename
                                                argv[3] //outFilename
                                                );
                }
            }
            else if(mode == "solsMindist_solsMindistSorted"){
                solsMindist_solsMindistSorted(argv[2], //solsMindistFilename
                                              argv[3] //outFilename
                                              );
            }
            else if(mode == "fixtime"){
                runCompute4FingeredGraspsFixtime(argv[2], //submode
                                                 argv[3], //objFilename,
                                                 argv[4], //outFilename,
                                                 atof(argv[5]), //halfAngle,
                                                 atof(argv[6]) //timelimit
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
