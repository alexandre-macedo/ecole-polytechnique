#pragma once
#ifndef RT_SPHERE_H
#define RT_SPHERE_H

#include <iostream>

#include "rt_ray.h"
#include "rt_color.h"

class RtSphere
{
  public:
    RtSphere();
    ~RtSphere();
    RtSphere(RtVector, double, RtColor);
    friend std::ostream &operator<<(std::ostream &, const RtSphere &);
    RtVector getCenter();
    double getRadius();

  private:
    RtVector center_;
    RtColor color_;
    double radius_;
};

#endif /* RT_SPHERE_H */