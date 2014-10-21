#ifndef GRASP_H
#define GRASP_H

#include <Eigen/Dense>
#include <vector>
#include <set>
#include "SurfacePoint.h"

class Grasp
{
    public:
        Grasp();
        virtual ~Grasp();

        Grasp(unsigned int sp1, unsigned int sp2, unsigned int sp3, unsigned int sp4);
        Grasp(const std::vector<unsigned int> &_surfacePoints);

        std::vector<unsigned int> surfacePoints;

        //compare
        inline bool operator< (Grasp const& r) const {
            const Grasp& l = *this;
            return l.surfacePoints < r.surfacePoints ;
        }

    protected:
    private:
};

#endif // GRASP_H
