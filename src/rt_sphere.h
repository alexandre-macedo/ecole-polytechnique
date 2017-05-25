#pragma once
#ifndef RT_SPHERE_H
#define RT_SPHERE_H

#include <iostream>

#include "rt_color.h"
#include "rt_ray.h"

class RtSphere {
public:
  RtSphere();
  ~RtSphere();
  RtSphere(const RtVector &, double, const RtColor &);
  friend std::ostream &operator<<(std::ostream &, const RtSphere &);
  RtVector getCenter() const;
  double getRadius() const;
  RtColor getColor() const;

private:
  RtVector center_;
  RtColor color_;
  double radius_;
};

#endif /* RT_SPHERE_H */