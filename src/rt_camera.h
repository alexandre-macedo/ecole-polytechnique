#pragma once
#ifndef RT_CAMERA_H
#define RT_CAMERA_H

#include <iostream>

#include "rt_vector.h"

class RtCamera {
public:
  RtCamera();
  ~RtCamera();
  RtCamera(const RtVector &, const RtVector &, const RtVector &, double,
           double);
  friend std::ostream &operator<<(std::ostream &, const RtCamera &);
  RtVector getTarget() const;
  RtVector getEye() const;
  RtVector getUp() const;
  double getWidth() const;
  double getHeight() const;

private:
  RtVector eye_;
  RtVector target_;
  RtVector up_;
  struct {
    double w, h;
  } dimensions_;
};

#endif /* RT_CAMERA_H */