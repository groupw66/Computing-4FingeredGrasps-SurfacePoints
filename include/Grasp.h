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

        inline unsigned int operator[] (unsigned int i) const {
            return surfacePoints[i];
        }

        //compare
        inline bool operator< (Grasp const& r) const {
            const Grasp& l = *this;
            return l.surfacePoints < r.surfacePoints ;
        }

        inline const char* to_str() const {
            std::stringstream ss;
            ss << (*this)[0] << " " << (*this)[1] << " " << (*this)[2] << " " << (*this)[3] ;
            return ss.str().c_str();
        }

    protected:
    private:
};

#endif // GRASP_H
