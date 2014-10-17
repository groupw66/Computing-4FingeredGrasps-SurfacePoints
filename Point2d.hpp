#ifndef POINT2D_HPP_INCLUDED
#define POINT2D_HPP_INCLUDED

struct Point2d : std::pair<double, double>{

	Point2d(double _x,double _y) :
	std::pair<double, double>(_x,_y){}

	Point2d operator -(const Point2d &b)const{
		return Point2d(this->first-b.first, this->second-b.second);
	}

	inline double norm2()const{
		return hypot(this->first,this->second);
//		return sqrt(this->first*this->first + this->second*this->second);
	}
};

#endif // POINT2D_HPP_INCLUDED
