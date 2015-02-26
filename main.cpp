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
#include "MedialAxis.h"

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

void genMedialAxis(int argc,char *argv[]);

void computeMindist(int argc,char *argv[]);

int main(int argc,char *argv[])
{
    if(argc > 1){
        if(cmdOptionExists(argv, argv+argc, "synthesis"))
        {
            graspSynthesis(argc, argv);
        }
        else if(cmdOptionExists(argv, argv+argc, "medialaxis"))
        {
            genMedialAxis(argc, argv);
        }
        else if(cmdOptionExists(argv, argv+argc, "mindist"))
        {
            computeMindist(argc, argv);
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

    //// MODE
    //-randomgrasp
    bool isRandomgrasp = cmdOptionExists(argv, argv+argc, "-randomgrasp");
    std::uniform_int_distribution<int> randSurfacePoint(0,osp.surfacePoints.size()-1);

    //-spanfilter
    bool isSpanfilter = cmdOptionExists(argv, argv+argc, "-spanfilter");
    DaeHeuristicChecker daeHeuristicChecker(halfangle * M_PI / 180.d);

    //-concurrent
    bool isConcurrent = cmdOptionExists(argv, argv+argc, "-concurrent");

    //-naiveeenumerate
    bool isNaiveeenumerate = cmdOptionExists(argv, argv+argc, "-naiveeenumerate");

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
    bool isSizeAABB = cmdOptionExists(argv, argv+argc, "-sizeAABB");
    double sizeAABB = 1.0;
    if(isSizeAABB) sizeAABB = atof(getCmdOption(argv, argv + argc, "-sizeAABB"));
    Eigen::Vector3d startStepPoint = osp.minAABB + (osp.minAABB-osp.cm)*(sizeAABB-1.);
    Eigen::Vector3d endStepPoint = osp.maxAABB + (osp.maxAABB-osp.cm)*(sizeAABB-1.);
    Eigen::Vector3d diffBox = endStepPoint - startStepPoint;
    double volumnBox = fabs(diffBox.x() * diffBox.y() * diffBox.z());
    double stepLength = pow(volumnBox/nStepPoints, 1.d/3.d);
    bool isStepB = cmdOptionExists(argv, argv+argc, "-stepb");
    if(isStepB){
        startStepPoint -= Eigen::Vector3d(stepLength/2, stepLength/2, stepLength/2);
        endStepPoint += Eigen::Vector3d(stepLength/2, stepLength/2, stepLength/2);
    }
    Eigen::Vector3d currentStepPoint = startStepPoint;

    //medialpoint
    MedialAxis medialAxis;
    std::vector<bool> usedMedialpoint;
    usedMedialpoint.resize(osp.surfacePoints.size());
    Eigen::Vector3d diffAABB = osp.maxAABB - osp.minAABB;
    double minwidth = std::min(std::abs(diffAABB.x()), std::abs(diffAABB.y()));
    minwidth = std::min(minwidth, std::abs(diffAABB.z()));
    std::normal_distribution<double> rerandMedialPoint(0.d, minwidth*0.2);
    int countUsedMedialpoint = 0;

    //-medialpoint
    bool isMedialpoint = cmdOptionExists(argv, argv+argc, "-medialpoint");
    std::tuple<Eigen::Vector3d, double, double> medialPoint;

    //-allmedialpoints
    bool isAllmedialpoints = cmdOptionExists(argv, argv+argc, "-allmedialpoints");
    std::vector< std::tuple<Eigen::Vector3d, double, double> > allMedialPoints;
    std::vector<double> accRadius;
    std::uniform_real_distribution<double> randMedialPoint(0, 1);

    //-randbmode
    int randBMode = 1;
    if(cmdOptionExists(argv, argv+argc, "-randbmode")) randBMode = atoi(getCmdOption(argv, argv+argc, "-randbmode"));
    //-minradius
    double minradius = 0;
    if(cmdOptionExists(argv, argv+argc, "-minradius")) minradius = atof(getCmdOption(argv, argv+argc, "-minradius"));
    minradius *= minwidth;


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
    long long nTestedPoint = 0;
    long long nSols = 0;
    std::unordered_set<std::string> solsSet;
    solsSet.reserve(1000000);
    std::vector<std::tuple<Grasp, double, double> > sols;
    sols.reserve(1000000);
    std::vector<std::tuple<Eigen::Vector3d, int, int> > concurrentPoints;
    concurrentPoints.reserve(200000);
    std::vector<std::vector<std::tuple<Grasp, double, double> > > concurrentSols;
    concurrentPoints.reserve(200000);
    std::vector<std::tuple<double, double, double, double, double> > concurrentPointTimes;
    concurrentPointTimes.reserve(200000);

    tmr.start("all");
    if(isConcurrent){
        tmr.reset();
        if(isMedialpoint || isAllmedialpoints){
            tmr.start("initMedialPoints");
            medialAxis.setObject(osp);
            if(isAllmedialpoints){
                medialAxis.genMedialPoints(allMedialPoints, randBMode, minradius);
                usedMedialpoint.resize(allMedialPoints.size());
                accRadius.clear();
                accRadius.push_back(0);
                for(unsigned int i=0 ; i<allMedialPoints.size() ; ++i){
                    accRadius.push_back(accRadius[i] + std::abs(std::get<1>(allMedialPoints[i])));
                }
                decltype(randMedialPoint.param()) new_range(0, accRadius[accRadius.size()-1]);
                randMedialPoint.param(new_range);
            }
            tmr.pause("initMedialPoints");
        }

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
            else if(isMedialpoint){
                tmr.start("genMedialPoint");
                Eigen::Vector3d pB;
                while(true){
                    if(!isRerandom && countUsedMedialpoint > randSurfacePoint.max()){
                        ++countUsedMedialpoint;
                        break;
                    }
                    unsigned int iA = randSurfacePoint(rng);
                    if(!isRerandom && usedMedialpoint[iA])
                        continue;
                    usedMedialpoint[iA] = true;
                    ++countUsedMedialpoint;
                    if(medialAxis.genMedialPoint(medialPoint, pB, iA, -1, randBMode, minradius) == 0 || isRerandom)
                        break;
                }
                if(!isRerandom && countUsedMedialpoint > randSurfacePoint.max()+1)
                    break;
                concurrentPoint = std::get<0>(medialPoint);
                tmr.pause("genMedialPoint");
            }
            else if(isAllmedialpoints){
                if(!isRerandom && countUsedMedialpoint > randSurfacePoint.max())
                    break;
                int iMedialPoint;
                do{
                    auto lower = std::lower_bound(accRadius.begin(), accRadius.end(), randMedialPoint(rng));
                    iMedialPoint = lower - accRadius.begin();
                }while(!isRerandom && usedMedialpoint[iMedialPoint]);
                usedMedialpoint[iMedialPoint] = true;
                medialPoint = allMedialPoints[iMedialPoint];
                concurrentPoint = std::get<0>(medialPoint);
            }
            else{
                std::cout << "Please specify concurrent points sampling method." << std::endl;
                return;
            }

            if(isRerandom){
                if(isMedialpoint || isAllmedialpoints){
                decltype(rerandMedialPoint.param()) new_range(0, std::get<1>(medialPoint)/rerandomSdDivider );
                rerandMedialPoint.param(new_range);
                    concurrentPoint += Eigen::Vector3d(rerandMedialPoint(rng), rerandMedialPoint(rng), rerandMedialPoint(rng));
                }
                else{
                    concurrentPoint += Eigen::Vector3d(rerandNX(rng), rerandNY(rng), rerandNZ(rng));
                }
            }

            tmr.start("one iterate");

            nTestedPoint++;
            concurrentSols.resize(nTestedPoint);
            concurrentSols[nTestedPoint-1].reserve(50000);

            tmr.start("pointInConesFilter");
            std::vector<unsigned int> filteredPointIds;
            Compute4FingeredGrasps::pointInConesFilter(filteredPointIds, osp.surfacePoints, concurrentPoint, halfangle);
            tmr.pause("pointInConesFilter");

            tmr.start("findEquilibriumGrasps");
            if(filteredPointIds.size() >= 4){
                if(isNaiveeenumerate){
                    Compute4FingeredGrasps::findEquilibriumGrasps_naive(
                                            concurrentSols[nTestedPoint-1], solsSet,
                                            isMindist, isUniquesol, tmr, timelimit, halfangle,
                                            filteredPointIds, concurrentPoint, osp.surfacePoints);
                }
                else{
                    Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(
                                            concurrentSols[nTestedPoint-1], solsSet,
                                            isMindist, isUniquesol, tmr, timelimit, halfangle,
                                            filteredPointIds, concurrentPoint, osp.surfacePoints);
                }
            }
            tmr.pause("findEquilibriumGrasps");
            tmr.pause("one iterate");
            concurrentPoints.push_back(std::make_tuple(concurrentPoint, filteredPointIds.size(), concurrentSols[nTestedPoint-1].size()));
            concurrentPointTimes.push_back(std::make_tuple(tmr.getSecLastLap("one iterate"),
                tmr.getSecLastLap("pointInConesFilter"), tmr.getSecLastLap("findEquilibriumGrasps"),
                tmr.getSecLastLap("make range tree"), tmr.getSecLastLap("points pairing")));
            nSols += concurrentSols[nTestedPoint-1].size();
            if(isNosol){
                concurrentSols[nTestedPoint-1].clear();
            }
            concurrentSols[nTestedPoint-1].shrink_to_fit();
        }
        //write output
        outFile << "nSols " << nSols << "\n";
        outFile << "nTestedPoint " << nTestedPoint << "\n";
        for(unsigned int i=0 ; i<concurrentPoints.size() ; ++i){
            auto conPoint = concurrentPoints[i];
            auto conPointTime = concurrentPointTimes[i];
            outFile << "cp " << std::get<0>(conPoint).x() << " " << std::get<0>(conPoint).y() << " " << std::get<0>(conPoint).z() << " "
                    << std::get<1>(conPoint) << " " << std::get<2>(conPoint) << "\n";
            outFile << "cpTime " << std::get<0>(conPointTime) << " " << std::get<1>(conPointTime) << " "
                    << std::get<2>(conPointTime) << " " << std::get<3>(conPointTime) << " " << std::get<4>(conPointTime) << "\n";
            for(auto sol : concurrentSols[i]){
                outFile << "g " << std::get<0>(sol).to_str() << " " << std::get<2>(sol) << " " << std::get<1>(sol) << "\n";
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
        outFile << "nSols " << nSols << "\n";
        outFile << "nTestedPoint " << nTestedPoint << "\n";
        for(auto sol : sols){
            outFile << "g " << std::get<0>(sol).to_str() << " " << std::get<2>(sol) << " " << std::get<1>(sol) << "\n";
        }
    }
    std::cout << filename << std::endl;
    std::cout << "nSols : " << nSols << ", nTestedPoint : " << nTestedPoint << std::endl;
    tmr.pause("all");
    outFile << "-------------------\n";
    outFile << tmr.strStopwatch();
    outFile.close();
}


void genMedialAxis(int argc,char *argv[])
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

    int nnMode = 1;
    if(cmdOptionExists(argv, argv+argc, "-nnMode")) {
        nnMode = atoi(getCmdOption(argv, argv + argc, "-nnMode"));
    }

    int randBMode = 1;
    if(cmdOptionExists(argv, argv+argc, "-randBMode")) {
        randBMode = atoi(getCmdOption(argv, argv + argc, "-randBMode"));
    }

    Timer tmr;
    tmr.start("genAllMedialPoints");
    MedialAxis medialAxis(osp, nnMode);
    std::vector< std::tuple<Eigen::Vector3d, double, double> > medialPoints;
    medialAxis.genMedialPoints(medialPoints, randBMode);
    tmr.pause("genAllMedialPoints");
    for(unsigned int i=0 ; i< medialPoints.size() ; ++i){
        auto mp = medialPoints[i];
        outFile << "mp " << std::get<0>(mp).x() << " " << std::get<0>(mp).y() << " " << std::get<0>(mp).z() << " "
                    << std::get<1>(mp) << " " << std::get<2>(mp) << "\n";
    }
    std::cout << filename << std::endl;
    std::cout << "genAllMedialPoints : " << tmr.getSec("genAllMedialPoints") << std::endl;
    outFile << "-------------------\n";
    outFile << tmr.strStopwatch();
    outFile.close();
}

void computeMindist(int argc, char* argv[])
{
    //open input file
    std::ifstream inFile;
    if(cmdOptionExists(argv, argv+argc, "-i")) {
        char * filename = getCmdOption(argv, argv + argc, "-i");
        inFile.open(filename);
        if(!inFile.is_open()){
            std::cout << "! Can't open " << filename << std::endl;
            return;
        }
    }
    else {
        std::cout << "not found output flag -o" << std::endl;
        return;
    }

    //open object file
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

    Timer tmr;
    std::vector<std::string> outStrings;
    std::vector<double> mindists;
    tmr.start("all");
    while(!inFile.eof())
    {
        std::string line;
        getline(inFile, line);
        std::stringstream ss(line);
        std::string tmp;
        ss >> tmp;
        if(tmp == "g"){
            int a, b, c, d;
            ss >> a >> b >> c >> d;
            tmr.start("getMindist_ZC");
            double mindist = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b],
                                                         osp.surfacePoints[c], osp.surfacePoints[d],
                                                         Eigen::Vector3d(0,0,0), halfangle);
            tmr.pause("getMindist_ZC");
            char tmpOut[1000];
            sprintf(tmpOut, "g %d %d %d %d %lf", a, b, c, d, mindist);
            outStrings.push_back(std::string(tmpOut));
            mindists.push_back(mindist);
        }
        else{
            outStrings.push_back(line);
        }
    }
    double sum = std::accumulate(std::begin(mindists), std::end(mindists), 0.0);
    double m =  sum / mindists.size();

    double accum = 0.0;
    std::for_each (std::begin(mindists), std::end(mindists), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (mindists.size()-1));
    outFile << "mean " << m << "\n";
    outFile << "stdev " << stdev << "\n";

    for(unsigned int i=0; i<outStrings.size() ; ++i){
        outFile << outStrings[i] << "\n";
    }
    std::cout << filename << std::endl;
    std::cout << "mean " << m << " ; stdev " << stdev << "\n";
    tmr.pause("all");
    outFile << "-------------------\n";
    outFile << tmr.strStopwatch();
    outFile.close();
    inFile.close();
}

