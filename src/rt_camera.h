#pragma once
#ifndef RT_CAMERA_H
#define RT_CAMERA_H

#include <iostream>

#include "rt_vector.h"

class RtCamera
{
  public:
    RtCamera();
    ~RtCamera();
    RtCamera(RtVector, RtVector, RtVector, double, double);
    friend std::ostream &operator<<(std::ostream &, const RtCamera &);
  private:
    RtVector eye_;
    RtVector target_;
    RtVector up_;
    struct{double w, h;} dimensions_; 
};

#endif /* RT_CAMERA_H */