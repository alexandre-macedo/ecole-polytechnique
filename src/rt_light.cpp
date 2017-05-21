#include "rt_light.h"

// Make standard contructor and destructor later if you have a bug
// I did not do one here just to test

RtLight::RtLight(){};
RtLight::~RtLight(){};

RtLight::RtLight(const RtVector &pt, const RtColor &color)
    : point_(pt), color_(color) {}

std::ostream &operator<<(std::ostream &s, const RtLight &li) {
  s << "[" << li.point_ << ", " << li.color_ << "]";
  return s;
};

RtVector RtLight::getPoint() const { return point_; }

RtColor RtLight::getColor() const { return color_; }
