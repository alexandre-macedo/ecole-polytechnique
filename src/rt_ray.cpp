#include "rt_ray.h"


RtRay::RtRay(double x_, double y_, double z_, RtVector direction_){
    point = struct{
        double x = x_;
        double y = y_;
        double z = z_;
    }
    direction = direction_;
}

RtRay::print(){
    printf("Vector with origin (%lf, %lf, %lf) and direction ", point.x, point.y, point.z);
    direction.print();
    printf("\n");
}