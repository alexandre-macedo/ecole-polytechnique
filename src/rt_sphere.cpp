#include <stdexcept>
#include <cmath>

#include "rt_sphere.h"

RtSphere::RtSphere() {}

RtSphere::~RtSphere() {}

RtSphere::RtSphere(RtVector c, double rad, int r, int g, int b) : center_(c), radius_(rad), color_{r,g,b}
{
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
        throw std::runtime_error("Invalid color.");
    if (rad < 0)
        throw std::runtime_error("Negative radius.");
}

std::ostream &operator<<(std::ostream &s, const RtSphere &sp){
    s << "[" << sp.center_ << ", " << sp.radius_ << ", [" << sp.color_.r << ", " << sp.color_.g << ", " << sp.color_.b << "]]";
    return s;
};

bool RtSphere::intersectionWith(RtRay r, RtVector &i){
    RtVector dir = r.getDirection();
    dir = dir* (1./(dir.norm2())); // norm == 1 now
    RtVector p = r.getPoint();
    double d = ((dir.cross(center_-p)).norm2());
    
    if(d > radius_){
        return false;
    }
    
    double hip = (center_-p).norm2();
    double cateto = sqrt(hip*hip - d*d);
    double correction = sqrt(radius_*radius_ - d*d);
    // Updating intersection point 
    //std::cout << "p: " << p << " dir: " << dir << " cat: " << cateto << " dist: " << d << std::endl;;
    i = p + dir*(cateto-correction);

    return true;
}


