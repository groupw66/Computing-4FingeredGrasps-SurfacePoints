#ifndef OBJECT_HPP_INCLUDED
#define OBJECT_HPP_INCLUDED

#include<vector>
#include"PointNormal.hpp"

struct Object : std::vector<PointNormal>{
	double minx,miny,minz;
	double maxx,maxy,maxz;
	double xrange,yrange,zrange;
	Object(const std::vector<PointNormal> &points) :
	std::vector<PointNormal>(points){
		minx=miny=minz=DBL_MAX;
		maxx=maxy=maxz=DBL_MIN;
		for(const PointNormal &pn : points){
			minx=std::min(minx,pn.point.x());
			miny=std::min(miny,pn.point.y());
			minz=std::min(minz,pn.point.z());
			maxx=std::max(maxx,pn.point.x());
			maxy=std::max(maxy,pn.point.y());
			maxz=std::max(maxz,pn.point.z());
		}
		xrange=maxx-minx;
		yrange=maxy-miny;
		zrange=maxz-minz;
	}
};

#endif // OBJECT_HPP_INCLUDED
