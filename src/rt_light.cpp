#include "rt_light.h"

// Make standard contructor and destructor later if you have a bug 
// I did not do one here just to test

RtLight::RtLight(){};
RtLight::~RtLight(){};

RtLight::RtLight(RtVector pt, RtColor color) : point_(pt), color_(color) {}


std::ostream &operator<<(std::ostream &s, const RtLight &li){
    s << "[" << li.point_ << ", "<< li.color_ << "]";
    return s;
};
