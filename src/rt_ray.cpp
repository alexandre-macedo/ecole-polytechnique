#include "rt_ray.h"
#include <stdio.h>

RtRay::RtRay(double x_, double y_, double z_, RtVector &direction_){
    point.x= x_;
    point.y= y_;
    point.z= z_;
    direction= direction_;
}

RtRay::RtRay(){}

RtRay::~RtRay(){}

void RtRay::print(){
    printf("Vector with origin (%.3lf, %.3lf, %.3lf) and direction ", point.x, point.y, point.z);
    direction.print();
}