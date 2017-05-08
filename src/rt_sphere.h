#pragma once
#ifndef RT_SPHERE_H
#define RT_SPHERE_H

#include <iostream>

#include "rt_ray.h"

class RtSphere
{
  public:
    RtSphere();
    ~RtSphere();
    RtSphere(RtVector, double, int, int, int);
    friend std::ostream &operator<<(std::ostream &, const RtSphere &);
    
    // if there is an intersectiom, the RtVector received will be updated to it
    bool intersectionWith(RtRay, RtVector&); 

  private:
    RtVector center_;
    struct { int r, g, b;} color_;
    double radius_;
};

#endif /* RT_SPHERE_H */