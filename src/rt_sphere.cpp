#define DOUBLE_COMPARE 1.0e-6

#include <stdexcept>

#include "rt_sphere.h"

RtSphere::RtSphere() {}

RtSphere::~RtSphere() {}

RtSphere::RtSphere(const RtVector &center, double rad, const RtColor &color)
    : center_(center), radius_(rad), color_(color) {

  if (rad < 0)
    throw std::runtime_error("Negative radius.");
}

RtSphere::RtSphere(const RtVector &center, double rad, const RtColor &color,
                   double reflectionCoeficient)
    : center_(center), radius_(rad), color_(color),
      reflectionCoeficient_(reflectionCoeficient) {

  if (rad < 0)
    throw std::runtime_error("Negative radius.");
  if (reflectionCoeficient < 0.0 || reflectionCoeficient > 1.0)
    throw std::runtime_error("Invalid reflection coeficient.");
}

std::ostream &operator<<(std::ostream &s, const RtSphere &sp) {
  s << "[" << sp.center_ << ", " << sp.radius_ << ", " << sp.color_ << "]";
  return s;
};

bool RtSphere::operator==(const RtSphere &sp) const {
  return (this->center_.distanceTo(sp.center_) < DOUBLE_COMPARE) &&
         ((this->radius_ - sp.radius_) < DOUBLE_COMPARE);
};

RtVector RtSphere::getCenter() const { return center_; }

double RtSphere::getRadius() const { return radius_; }

RtColor RtSphere::getColor() const { return color_; }

void RtSphere::copyHere(RtSphere &other) {
  center_.copyHere(other.getCenter());
  color_.copyHere(other.getColor());
  radius_ = other.getRadius();
}
