#include <stdexcept>

#include "rt_sphere.h"

RtSphere::RtSphere() {}

RtSphere::~RtSphere() {}

RtSphere::RtSphere(const RtVector &center, double rad, const RtColor &color)
    : center_(center), radius_(rad), color_(color) {

  if (rad < 0)
    throw std::runtime_error("Negative radius.");
}

std::ostream &operator<<(std::ostream &s, const RtSphere &sp) {
  s << "[" << sp.center_ << ", " << sp.radius_ << ", " << sp.color_ << "]";
  return s;
};

RtVector RtSphere::getCenter() const { 
  return center_; 
}

double RtSphere::getRadius() const { 
  return radius_; 
}

RtColor RtSphere::getColor() const{
  return color_;
}
