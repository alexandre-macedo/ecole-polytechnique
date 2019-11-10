#pragma once
#ifndef RT_LIGHT_H
#define RT_LIGHT_H

#include <iostream>

#include "rt_color.h"
#include "rt_vector.h"

class RtLight {
public:
  RtLight();
  ~RtLight();
  RtLight(const RtVector &, const RtColor &);
  friend std::ostream &operator<<(std::ostream &, const RtLight &);
  RtVector getPoint() const;
  RtColor getColor() const;

private:
  RtVector point_;
  RtColor color_;
};

#endif /* RT_LIGHT_H */