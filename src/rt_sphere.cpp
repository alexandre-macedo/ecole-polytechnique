#include "rt_sphere.h"
#include <stdio.h>

RtSphere::RtSphere(double x_, double y_, double z_, double r_, Color c_){
    center.x = x_;
    center.y = y_;
    center.z = z_;
    radius = r_;
    color = c_;
}

void RtSphere::print(){
    printf("Sphere with center (%lf, %lf, %lf), radius = %lf and color ", center.x,
    center.y, center.z, radius);
    switch(color){
        case red : printf("red\n");
        case green: printf("green\n");
        case blue: printf("blue\n");
    }
}