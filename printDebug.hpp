#ifndef PRINTDEBUG_HPP_INCLUDED
#define PRINTDEBUG_HPP_INCLUDED

#include<cstring>

#define MAXLEN (512)
inline char* print(const Point3d &point, bool newLine=true){
	char *str=new char[MAXLEN];
	int len=sprintf(str,"%lf %lf %lf ",point.x(),point.y(),point.z());
	if(newLine) sprintf(str+len,"\n");
	return str;
}

inline char* print(const PointNormal &pn){
	char *str;
	str=strcat(print(pn.point,false),print(pn.normal));
	return str;
}

inline char* print(const Grasp &grasp){
	Grasp::iterator finger=grasp.begin();
	char *str=print(*finger);
	for(++finger;finger!=grasp.end();++finger){
		str=strcat(str,print(*finger));
	}
	return str;
}

#endif // PRINTDEBUG_HPP_INCLUDED
