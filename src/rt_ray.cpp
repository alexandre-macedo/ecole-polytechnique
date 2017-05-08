#include "rt_ray.h"

RtRay::RtRay() {}

RtRay::~RtRay() {}

RtRay::RtRay(RtVector point, RtVector direction) : point_(point), direction_(direction) {}

std::ostream &operator<<(std::ostream &s, const RtRay &r)
{
    s << "[" << r.point_ << ", " << r.direction_ << "]";
    return s;
}

RtVector RtRay::getDirection(){
    return this->direction_;
}

RtVector RtRay::getPoint(){
    return this->point_;
}

