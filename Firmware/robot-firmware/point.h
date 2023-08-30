#ifndef __POINT_H__
#define __POINT_H__

typedef struct point_s point;

struct point_s {
  float x, y;   
};

point point_add(point p1, point p2);
point point_sub(point p1, point p2);
point point_scale(point p1, float c);

float point_mag(point *p1);
float point_arg(point *p1);
float point_dot(point p1, point p2);

// Projects vector u onto vector v
point point_project(point u, point v);


#endif