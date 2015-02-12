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
#include <climits>

using namespace std;

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

bool canOpenFile(const char* filename)
{
    std::ifstream myFile(filename);
    if(!myFile.is_open()){
        std::cout << "! Can't open " << filename << std::endl;
        return false;
    }
    myFile.close();
    return true;
}

void graspSynthesis(int argc,char *argv[]);

int main(int argc,char *argv[])
{
    if(argc > 1){
        if(cmdOptionExists(argv, argv+argc, "synthesis"))
        {
            graspSynthesis(argc, argv);
        }
    }
    else{
        std::unordered_set<std::string> solsSet;
        solsSet.reserve(1000000);
        std::cout << solsSet.insert(Grasp(161, 190, 559, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 190, 659, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 191, 559, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 191, 659, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 191, 659, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 201, 213, 758).to_str()).second << std::endl;
        std::cout << solsSet.insert(Grasp(161, 213, 559, 758).to_str()).second << std::endl;
        std::cout << std::numeric_limits<long double>::digits10 << " " << std::numeric_limits<double>::digits10 << std::endl;
        return UnitTest::RunAllTests();
    }
    return 0;
}

void graspSynthesis(int argc,char *argv[])
{
    //open input file
    ObjectSurfacePoints osp;
    if(cmdOptionExists(argv, argv+argc, "-obj")) {
        char * filename = getCmdOption(argv, argv + argc, "-obj");
        if(!canOpenFile(filename)) return;
        OBJFile obj(filename);
        osp.open(obj);
    }
    else if(cmdOptionExists(argv, argv+argc, "-pnf")) {
        char * filename = getCmdOption(argv, argv + argc, "-pnf");
        if(!canOpenFile(filename)) return;
        PositionsNormalsFile obj(filename);
        osp.open(obj);
    }
    else {
        std::cout << "not found input flag -obj or -pnf" << std::endl;
        return;
    }

    //open output file
    char* filename;
    std::ofstream outFile;
    if(cmdOptionExists(argv, argv+argc, "-o")) {
        filename = getCmdOption(argv, argv + argc, "-o");
        outFile.open(filename);
        outFile.unsetf ( std::ios::floatfield );
        outFile.precision(std::numeric_limits<double>::digits10);
        if(!outFile.is_open()){
            std::cout << "! Can't open " << filename << std::endl;
            return;
        }
    }
    else {
        std::cout << "not found output flag -o" << std::endl;
        return;
    }

    //half angle
    double halfangle = 10.0d;
    if(cmdOptionExists(argv, argv+argc, "-halfangle")) {
        halfangle = atof(getCmdOption(argv, argv + argc, "-halfangle"));
    }

    //limit by ...
    double timelimit = 3.15569e7d;
    if(cmdOptionExists(argv, argv+argc, "-timelimit")) {
        timelimit = atof(getCmdOption(argv, argv + argc, "-timelimit"));
    }
    int pointslimit = INT_MAX;
    if(cmdOptionExists(argv, argv+argc, "-pointslimit")) {
        pointslimit = atoi(getCmdOption(argv, argv + argc, "-pointslimit"));
    }

    //-uniquesol
    bool isUniquesol = cmdOptionExists(argv, argv+argc, "-uniquesol");

    //-nosol
    bool isNosol = cmdOptionExists(argv, argv+argc, "-nosol");

    //-mindist
    bool isMindist = cmdOptionExists(argv, argv+argc, "-mindist");

    //-randomgrasp
    bool isRandomgrasp = cmdOptionExists(argv, argv+argc, "-randomgrasp");
    std::uniform_int_distribution<int> randSurfacePoint(0,osp.surfacePoints.size()-1);

    //-spanfilter
    bool isSpanfilter = cmdOptionExists(argv, argv+argc, "-spanfilter");
    DaeHeuristicChecker daeHeuristicChecker(halfangle * M_PI / 180.d);

    //-concurrent
    bool isConcurrent = cmdOptionExists(argv, argv+argc, "-concurrent");

    //-uniform
    bool isUniform = cmdOptionExists(argv, argv+argc, "-uniform");
    std::uniform_real_distribution<double> randUX(osp.minAABB.x(), osp.maxAABB.x());
    std::uniform_real_distribution<double> randUY(osp.minAABB.y(), osp.maxAABB.y());
    std::uniform_real_distribution<double> randUZ(osp.minAABB.z(), osp.maxAABB.z());

    //-nearcm
    bool isNearCM = cmdOptionExists(argv, argv+argc, "-nearcm");
    double sdDivider = 9;
    if(isNearCM) sdDivider = atof(getCmdOption(argv, argv + argc, "-nearcm"));
    Eigen::Vector3d mean = Eigen::Vector3d(0,0,0),
                    sd = (osp.maxAABB - osp.minAABB)/sdDivider;
    std::normal_distribution<double> randNX(mean.x(), sd.x());
    std::normal_distribution<double> randNY(mean.y(), sd.y());
    std::normal_distribution<double> randNZ(mean.z(), sd.z());

    //-prepoint
    bool isPrePoint = cmdOptionExists(argv, argv+argc, "-prepoint");
    std::vector<std::pair<Eigen::Vector3d, bool> > prePoints;
    if(isPrePoint){
        std::string filename = getCmdOption(argv, argv + argc, "-prepoint");
        std::ifstream prepointfile(filename);
        if(!prepointfile.is_open()){
            std::cout << "! Can't open " << filename << std::endl;
            return;
        }
        std::string line;
        getline(prepointfile, line); //nVertices
        int nVertices = atoi(line.c_str());
        for(int i=0 ; i < nVertices ; ++i)
        {
            getline(prepointfile, line);
            std::stringstream ss(line);
            double x, y ,z;
            ss >> x >> y >> z;
            Eigen::Vector3d vertex(x,y,z);
            prePoints.push_back(std::make_pair(vertex,false));
        }
        prepointfile.close();
    }
    std::uniform_int_distribution<int> randPrepoint(0,prePoints.size()-1);

    //-step
    bool isStep = cmdOptionExists(argv, argv+argc, "-step");
    int nStepPoints = 1000;
    if(isStep) nStepPoints = atoi(getCmdOption(argv, argv + argc, "-step"));
    Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
    double volumnAABB = fabs(diffAABB.x() * diffAABB.y() * diffAABB.z());
    double stepLength = pow(volumnAABB/nStepPoints, 1.d/3.d);
    Eigen::Vector3d startStepPoint = osp.minAABB;
    Eigen::Vector3d endStepPoint = osp.maxAABB;
    bool isStepB = cmdOptionExists(argv, argv+argc, "-stepb");
    if(isStepB){
        startStepPoint = osp.minAABB - Eigen::Vector3d(stepLength/2, stepLength/2, stepLength/2);
        endStepPoint = osp.maxAABB + Eigen::Vector3d(stepLength/2, stepLength/2, stepLength/2);
    }
    Eigen::Vector3d currentStepPoint = startStepPoint;

    //-rerandom
    bool isRerandom = cmdOptionExists(argv, argv+argc, "-rerandom");
    double rerandomSdDivider = 18;
    if(isRerandom) rerandomSdDivider = atof(getCmdOption(argv, argv + argc, "-rerandom"));
    double rerandomSd = (osp.maxAABB - osp.minAABB).norm()/rerandomSdDivider;
    std::normal_distribution<double> rerandNX(0.d, rerandomSd);
    std::normal_distribution<double> rerandNY(0.d, rerandomSd);
    std::normal_distribution<double> rerandNZ(0.d, rerandomSd);

    ///////////////////////////////////////////////////
    Timer tmr;
    std::random_device rd;
    std::default_random_engine rng(rd());
    int nTestedPoint = 0;
    int nSols = 0;
    std::unordered_set<std::string> solsSet;
    solsSet.reserve(1000000);
    std::vector<std::tuple<Grasp, double, double> > sols;
    sols.reserve(1000000);
    std::vector<std::tuple<Eigen::Vector3d, int, int> > concurrentPoints;
    concurrentPoints.reserve(200000);
    std::vector<std::vector<std::tuple<Grasp, double, double> > > concurrentSols;
    concurrentPoints.reserve(200000);

    tmr.start("all");
    if(isConcurrent){
        tmr.reset();
        while(tmr.elapsed() < timelimit && nTestedPoint < pointslimit){
            Eigen::Vector3d concurrentPoint;
            if(isNearCM){
                concurrentPoint = Eigen::Vector3d(randNX(rng), randNY(rng), randNZ(rng));
            }
            else if(isPrePoint){
                int iPrePoint = randPrepoint(rng);
                concurrentPoint = prePoints[iPrePoint].first;

            }
            else if(isUniform){
                concurrentPoint = Eigen::Vector3d(randUX(rng), randUY(rng), randUZ(rng));
            }
            else if(isStep){
                if(currentStepPoint.z() > endStepPoint.z()){
                    currentStepPoint.z() = startStepPoint.z();
                    currentStepPoint.y() += stepLength;
                }
                if(currentStepPoint.y() > endStepPoint.y()){
                    currentStepPoint.y() = startStepPoint.y();
                    currentStepPoint.x() += stepLength;
                }
                if(currentStepPoint.x() > endStepPoint.x()){
                    break;
                }
                concurrentPoint = currentStepPoint;
                currentStepPoint.z() += stepLength;
            }
            else{
                std::cout << "Please specify concurrent points sampling method." << std::endl;
                return;
            }

            if(isRerandom){
                concurrentPoint += Eigen::Vector3d(rerandNX(rng), rerandNY(rng), rerandNZ(rng));
            }

            nTestedPoint++;
            concurrentSols.resize(nTestedPoint);
            concurrentSols[nTestedPoint-1].reserve(50000);

            tmr.start("pointInConesFilter");
            std::vector<unsigned int> filteredPointIds;
            Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, concurrentPoint, halfangle);
            tmr.pause("pointInConesFilter");

            tmr.start("findEquilibriumGrasps_forceDual");
            if(filteredPointIds.size() >= 4){
                Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                            concurrentSols[nTestedPoint-1], solsSet,
                                            isMindist, isUniquesol, tmr, timelimit, halfangle,
                                            filteredPointIds, concurrentPoint, osp.surfacePoints);
            }
            tmr.pause("findEquilibriumGrasps_forceDual");
            concurrentPoints.push_back(std::make_tuple(concurrentPoint, filteredPointIds.size(), concurrentSols[nTestedPoint-1].size()));
            nSols += concurrentSols[nTestedPoint-1].size();
            if(isNosol){
                concurrentSols[nTestedPoint-1].clear();
            }
            concurrentSols[nTestedPoint-1].shrink_to_fit();
        }
        //write output
        outFile << nSols << " " << nTestedPoint << "\n";
        for(unsigned int i=0 ; i<concurrentPoints.size() ; ++i){
            auto conPoint = concurrentPoints[i];
            outFile << std::get<0>(conPoint).x() << " " << std::get<0>(conPoint).y() << " " << std::get<0>(conPoint).z() << " "
                    << std::get<1>(conPoint) << " " << std::get<2>(conPoint) << "\n";
            for(auto sol : concurrentSols[i]){
                outFile << std::get<0>(sol).to_str() << " " << std::get<1>(sol) << " " << std::get<2>(sol) << "\n";
            }
        }

    }
    else if(isRandomgrasp){
        tmr.reset();
        while(tmr.elapsed() < timelimit && nTestedPoint < pointslimit){
            int a = randSurfacePoint(rng),
                b = randSurfacePoint(rng),
                c = randSurfacePoint(rng),
                d = randSurfacePoint(rng);
            if(a==b || a==c || a==d || b==c || b==d || c==d)
                continue;
            nTestedPoint++;
            Grasp grasp(a,b,c,d);
            if(isSpanfilter){
                tmr.start("daeHeuristicChecker");
                bool passFilter = daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b],
                                                                     osp.surfacePoints[c], osp.surfacePoints[d]);
                tmr.pause("daeHeuristicChecker");
                if(!passFilter)
                    continue;
            }
            tmr.start("isFC_ZC");
            bool isFC = ForceClosure::isFC_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                 osp.surfacePoints[c], osp.surfacePoints[d],
                                                 Eigen::Vector3d(0,0,0), halfangle);
            tmr.pause("isFC_ZC");
            if(!isFC)
                continue;
            if(isUniquesol){
                tmr.start("unique sol check");
                bool isUnique = solsSet.insert(grasp.to_str()).second;
                tmr.pause("unique sol check");
                if(!isUnique)
                    continue;
            }
            double mindist = -1;
            if(isMindist){
                tmr.start("compute mindist");
                mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                         osp.surfacePoints[c], osp.surfacePoints[d],
                                                         Eigen::Vector3d(0,0,0), halfangle);
                tmr.pause("compute mindist");
            }
            if(tmr.elapsed() < timelimit){
                sols.push_back(std::make_tuple(grasp, tmr.elapsed(), mindist));
            }
        }
        nSols = sols.size();
        //write output
        outFile << nSols << " " << nTestedPoint << "\n";
        for(auto sol : sols){
            outFile << std::get<0>(sol).to_str() << " " << std::get<1>(sol) << " " << std::get<2>(sol) << "\n";
        }
    }
    std::cout << filename << std::endl;
    std::cout << "nSols : " << nSols << ", nTestedPoint : " << nTestedPoint << std::endl;
    tmr.pause("all");
    outFile << "-------------------\n";
    outFile << tmr.strStopwatch();
    outFile.close();
}
