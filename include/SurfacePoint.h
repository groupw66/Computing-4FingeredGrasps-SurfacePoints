#ifndef SURFACEPOINT_H
#define SURFACEPOINT_H

#include <Eigen/Dense>
#include <tuple>

class SurfacePoint
{
    public:
        SurfacePoint();
        virtual ~SurfacePoint();
        SurfacePoint(Eigen::Vector3d _position, Eigen::Vector3d _normal){
            position = _position;
            normal = _normal.normalized();
        }

        inline void operator= (SurfacePoint const& r) {
            this->position = r.position;
            this->normal = r.normal;
        }

        //compare
        inline bool operator< (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple < rtuple ;
        }

        inline bool operator<= (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple <= rtuple ;
        }

        inline bool operator== (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple == rtuple ;
        }

        inline bool operator>= (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple >= rtuple ;
        }

        inline bool operator> (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple > rtuple ;
        }

        inline bool operator!= (SurfacePoint const& r) const {
            const SurfacePoint& l = *this;
            auto ltuple = std::make_tuple(l.position.x(), l.position.y(), l.position.z(), l.normal.x(), l.normal.y(), l.normal.z());
            auto rtuple = std::make_tuple(r.position.x(), r.position.y(), r.position.z(), r.normal.x(), r.normal.y(), r.normal.z());
            return ltuple != rtuple ;
        }

        Eigen::Vector3d position;
        Eigen::Vector3d normal;
    protected:
    private:
};

#endif // SURFACEPOINT_H
