#pragma once
#ifndef RT_IMAGE_H
#define RT_IMAGE_H

#include <vector>

#include "rt_camera.h"
#include "rt_color.h"
#include "rt_light.h"
#include "rt_scene.h"

class RtImage {
public:
  RtImage();
  ~RtImage();
  RtImage(const int, const int);
  int getHeight() const;
  int getWidth() const;
  friend std::ostream &operator<<(std::ostream &, const RtImage &);
  std::vector<std::vector<RtColor>>& getImage();

private:
  const int height_ = 0;
  const int width_ = 0;
  std::vector<std::vector<RtColor>> image_;
};

#endif /* RT_IMAGE_H */