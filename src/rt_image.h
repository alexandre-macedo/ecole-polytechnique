#pragma once
#ifndef RT_IMAGE_H
#define RT_IMAGE_H

#include <vector>

#include "rt_color.h"

class RtImage {
public:
  RtImage();
  ~RtImage();
  RtImage(int, int);
  
  private:
  int height_,width_;
  std::vector<std::vector< RtColor>> image_;
};

#endif /* RT_IMAGE_H */