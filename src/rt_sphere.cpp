#include "rt_sphere.h"
#include <stdio.h>

RtSphere::RtSphere(double x_, double y_, double z_, double rad_, int r_, int g_, int b_){
    center.x = x_;
    center.y = y_;
    center.z = z_;
    radius = rad_;
    // Initializing colors:
    color.red = r_;
    color.green = g_;
    color.blue = b_;
}

void RtSphere::print(){
    printf("Sphere with center (%.3lf, %.3lf, %.3lf), radius = %.3lf and color ", center.x,
    center.y, center.z, radius);
    printf("(red, green, blue) = (%d, %d, %d)\n", color.red, color.green, color.blue);
}


