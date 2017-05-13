#include <stdexcept>

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

RtVector RtSphere::getCenter(){
    return center_;
}

double RtSphere::getRadius(){
    return radius_;
}


