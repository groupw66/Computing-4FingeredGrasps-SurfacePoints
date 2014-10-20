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
        Grasp(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4);
        Grasp(const std::vector<SurfacePoint> &_surfacePoints);

        std::vector<SurfacePoint> surfacePoints;

        //compare
        inline bool operator< (Grasp const& r) const {
            const Grasp& l = *this;
            return l.surfacePoints < r.surfacePoints ;
        }

    protected:
    private:
};

#endif // GRASP_H
