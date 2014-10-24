#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "mVect.h"
#include <stdio.h>
#include <string.h>

typedef enum drawModeTAG { draw_cone, draw_vector } drawMode;


class SchemeOutput 
{
public:
  SchemeOutput(const char* filename,bool clearPart = true,bool append = false) { 
    r = 0;g = 255; b = 0;
    strcpy(this->filename,filename);
    if (append)
      fp = fopen(filename,"a");
    else
      fp = fopen(filename,"w");
    if (clearPart) addPartClear();
  }
  ~SchemeOutput() { fclose(fp); }

  void setColor(double R,double G,double B) {
    r = R;
    b = B;
    g = G;
  }
  void flush() {
    fclose(fp);
    fp = fopen(filename,"a");
  }

  void close() { fclose(fp); }

  void print(const char* st) { fprintf(fp,st); }
  void addPartClear() { fprintf(fp,"(part:clear)\n"); }
  void print(mvPoint3d p);
  void print_line(mvPoint3d p1,mvPoint3d p2);
  void print_point_vector(mvPoint3d p,mvPoint3d v);
  void print_block(mvPoint3d corner1,mvPoint3d corner2);
  void print_wire_block(mvPoint3d corner1,mvPoint3d corner2);
  void print_spherical(mvPoint3d p1,mvPoint3d p2);
  void print_cone(mvPoint3d pos,mvPoint3d norm,double distance,double halfAngle);
  void print_sphere(mvPoint3d origin,double radius);
  void print_plane(mvPoint3d position,mvPoint3d normal);

private:
  char filename[1000];
  double r,g,b;
  FILE *fp;
};

#endif