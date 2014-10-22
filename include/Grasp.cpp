#include "Grasp.h"

Grasp::Grasp()
{
    //ctor
}

Grasp::~Grasp()
{
    //dtor
}

Grasp::Grasp(unsigned int sp1, unsigned int sp2, unsigned int sp3, unsigned int sp4)
{
    surfacePoints.clear();
    surfacePoints.reserve(4);
    surfacePoints.push_back(sp1);
    surfacePoints.push_back(sp2);
    surfacePoints.push_back(sp3);
    surfacePoints.push_back(sp4);
    std::sort(surfacePoints.begin(), surfacePoints.end());
    //cm = _cm;
}

Grasp::Grasp(const std::vector<unsigned int>& _surfacePoints)
{
    surfacePoints.clear();
    surfacePoints.reserve(_surfacePoints.size());
    surfacePoints.insert(surfacePoints.end(), _surfacePoints.begin(), _surfacePoints.end());
    std::sort(surfacePoints.begin(), surfacePoints.end());
    //cm = _cm;
}
