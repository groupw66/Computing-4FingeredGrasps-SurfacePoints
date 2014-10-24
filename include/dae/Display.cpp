#include "Display.h"

void SchemeOutput::print(mvPoint3d p) {
  fprintf(fp,"(entity:set-color (solid:sphere %f %f %f 0.01) (color:rgb %f %f %f))\n",
    p.x,
    p.y,
    p.z,
    r,g,b);
}

void SchemeOutput::print_line(mvPoint3d p1,mvPoint3d p2)
{
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",
    p1.x,p1.y,p1.z,
    p2.x,p2.y,p2.z,
    r,g,b);
}

void SchemeOutput::print_point_vector(mvPoint3d p,mvPoint3d v) 
{
  mvPoint3d p2;
  p2 = p + v;
  print(p);
  print_line(p,p2);
}

void SchemeOutput::print_block(mvPoint3d corner1,mvPoint3d corner2) {
  fprintf(fp,"(entity:set-color (solid:block (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",
    corner1.x,
    corner1.y,
    corner1.z,
    corner2.x,
    corner2.y,
    corner2.z,r,g,b);
}

void SchemeOutput::print_wire_block(mvPoint3d corner1,mvPoint3d corner2) {
  double x1 = corner1.x;
  double y1 = corner1.y;
  double z1 = corner1.z;
  double x2 = corner2.x;
  double y2 = corner2.y;
  double z2 = corner2.z;
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y1,z1,x2,y1,z1,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y1,z1,x1,y2,z1,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y1,z1,x1,y1,z2,r,g,b);

  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y2,z1,x1,y2,z1,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y2,z1,x2,y1,z1,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y2,z1,x2,y2,z2,r,g,b);

  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y2,z2,x2,y2,z2,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y2,z2,x1,y1,z2,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x1,y2,z2,x1,y2,z1,r,g,b);

  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y1,z2,x1,y1,z2,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y1,z2,x2,y2,z2,r,g,b);
  fprintf(fp,"(entity:set-color (edge:linear (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",x2,y1,z2,x2,y1,z1,r,g,b);
}

void SchemeOutput::print_spherical(mvPoint3d p1,mvPoint3d p2)
{
  fprintf(fp,"(entity:set-color (edge:circular-center-rim (position 0 0 0) (position %f %f %f) (position %f %f %f)) (color:rgb %f %f %f))\n",
    p1.x,p1.y,p1.z,
    p2.x,p2.y,p2.z,
    r,g,b);
  print(p1);
  print(p2);
}

void SchemeOutput::print_sphere(mvPoint3d origin,double radius) 
{
  fprintf(fp,"(entity:set-color (solid:sphere %f %f %f %f) (color:rgb %f %f %f))\n",
    origin.x,
    origin.y,
    origin.z,
    radius,
    r,g,b);
}


void SchemeOutput::print_cone(mvPoint3d pos,mvPoint3d norm,double distance,double halfAngle) {
  mvPoint3d p2;
  p2 = pos + (norm.normalize() * distance);
  double rad = distance * tan(halfAngle);
  fprintf(fp,"(entity:set-color (solid:cone (position %f %f %f) (position %f %f %f) %f 0) (color:rgb %f %f %f))\n",
    p2.x,
    p2.y,
    p2.z,
    pos.x,
    pos.y,
    pos.z,
    rad,
    r,g,b);
}

void SchemeOutput::print_plane(mvPoint3d position,mvPoint3d normal)
{
  fprintf(fp,"(entity:set-color (face:plane (position %f %f %f) 200 200 (gvector %f %f %f)) (color:rgb %f %f %f))\n",
    position.x,
    position.y,
    position.z,
    normal.x,
    normal.y,
    normal.z,
    r,g,b);
  fprintf(fp,"(entity:set-color (face:plane (position %f %f %f) -200 200 (gvector %f %f %f)) (color:rgb %f %f %f))\n",
    position.x,
    position.y,
    position.z,
    normal.x,
    normal.y,
    normal.z,
    r,g,b);
  fprintf(fp,"(entity:set-color (face:plane (position %f %f %f) 200 -200 (gvector %f %f %f)) (color:rgb %f %f %f))\n",
    position.x,
    position.y,
    position.z,
    normal.x,
    normal.y,
    normal.z,
    r,g,b);
  fprintf(fp,"(entity:set-color (face:plane (position %f %f %f) -200 -200 (gvector %f %f %f)) (color:rgb %f %f %f))\n",
    position.x,
    position.y,
    position.z,
    normal.x,
    normal.y,
    normal.z,
    r,g,b);
}
