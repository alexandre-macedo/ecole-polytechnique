#include "rt_ray.h"

RtRay::RtRay() {}

RtRay::~RtRay() {}

RtRay::RtRay(RtVector origin, RtVector direction)
    : origin_(origin), direction_(direction) {}

std::ostream &operator<<(std::ostream &s, const RtRay &r) {
  s << "[" << r.origin_ << ", " << r.direction_ << "]";
  return s;
}

RtVector RtRay::getDirection() const { return this->direction_; }

RtVector RtRay::getOrigin() const { return this->origin_; }
