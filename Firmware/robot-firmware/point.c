#include <math.h>
#include "point.h"

point point_add(point p1, point p2){
  point p = {p1.x + p2.x, p2.y + p1.y};
  return p;
}

point point_scale(point p1, float c){
  point p;
  p.x = p1.x * c;
  p.y = p1.y * c;
  return p;
}

point point_sub(point p1, point p2){
  point p;
  p.x = p1.x - p2.x;
  p.y = p1.y - p2.y;
  return p;
}

float point_mag(point *p1){
  return sqrtf(p1->x * p1->x + p1->y * p1->y);
}

float point_arg(point *p1){
  return atan2f(p1->y, p1->x);
}

float point_dot(point p1, point p2){
  return p1.x * p2.x + p1.y * p2.y;
}

point point_project(point u, point v){
  // m = u.v / ||v|| ^ 2
  float m = point_dot(u, v) / point_dot(v, v);

  return point_scale(v, m);
}