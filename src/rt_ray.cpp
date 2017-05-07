#include "rt_ray.h"

RtRay::RtRay(double x, double y, double z, RtVector &direction) : point_{x,y,z}, direction_(direction) {}

RtRay::RtRay() {}

RtRay::~RtRay() {}

void RtRay::print()
{
    std::cout << "Ray with origin (" << point_.x << "," << point_.y << "," << point_.z << ") and direction " << direction_ << std::endl;
}

std::ostream &operator<<(std::ostream &s, const RtRay &r)
{
    s << "[(" << r.point_.x << ", " << r.point_.y << ", " << r.point_.z << "), " << r.direction_ << "]";
    return s;
}
