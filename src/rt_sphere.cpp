#include <stdexcept>

#include "rt_sphere.h"

RtSphere::RtSphere() {}

RtSphere::~RtSphere() {}

RtSphere::RtSphere(RtVector center, double rad, RtColor color) : center_(center), radius_(rad), color_(color)
{
    
    if (rad < 0)
        throw std::runtime_error("Negative radius.");
}

std::ostream &operator<<(std::ostream &s, const RtSphere &sp){
    s << "[" << sp.center_ << ", " << sp.radius_ << ", [" << sp.color_.getR() << ", " << sp.color_.getG() << ", " << sp.color_.getB() << "]]";
    return s;
};

RtVector RtSphere::getCenter(){
    return center_;
}

double RtSphere::getRadius(){
    return radius_;
}


