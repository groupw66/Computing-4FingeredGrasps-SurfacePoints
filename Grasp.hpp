#ifndef GRASP_HPP_INCLUDED
#define GRASP_HPP_INCLUDED

#include<set>

struct Grasp : std::set<PointNormal>{
//	PointNormal f1,f2,f3,f4;
//	std::set<PointNormal> fingers;
//	Grasp(PointNormal f1,PointNormal f2,PointNormal f3,PointNormal f4):fingers{f1,f2,f3,f4}{}
	Grasp(const PointNormal &f1,const PointNormal &f2,const PointNormal &f3,const PointNormal &f4) :
	std::set<PointNormal>{f1,f2,f3,f4}{}
//	bool operator <(const Grasp &b)const{
////		return (f1<b.f1) || (f1==b.f1&&f2<b.f2) || (f1==b.f1&&f2==b.f2&&f3<b.f3) || (f1==b.f1&&f2==b.f2&&f3==b.f3&&f4<b.f4);
//		return fingers<b.fingers;
//	}
};

#endif // GRASP_HPP_INCLUDED
