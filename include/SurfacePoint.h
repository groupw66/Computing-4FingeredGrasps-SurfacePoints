#ifndef SURFACEPOINT_H
#define SURFACEPOINT_H

#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include "Wrench.h"

#define DEFAULT_N_PYRAMID_SIDE 24 // 15 degree

class SurfacePoint
{
    public:
        Eigen::Vector3d position;
        Eigen::Vector3d normal;

        SurfacePoint();
        virtual ~SurfacePoint();

        SurfacePoint(Eigen::Vector3d _position, Eigen::Vector3d _normal){
            position = _position;
            normal = _normal.normalized();
        }

        std::vector<Eigen::Vector3d> getFrictionCone(double halfAngle = 10.d, int nPyramidSide = DEFAULT_N_PYRAMID_SIDE);
        std::vector<Wrench> getWrenchCone(Eigen::Vector3d cm = Eigen::Vector3d(0,0,0), double halfAngle = 10.d, int nPyramidSide = DEFAULT_N_PYRAMID_SIDE);

        inline void operator= (SurfacePoint const& r) {
            this->position = r.position;
            this->normal = r.normal;
        }

        inline bool operator< (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple < rtuple ;
        }
    protected:
    private:
};

#endif // SURFACEPOINT_H
