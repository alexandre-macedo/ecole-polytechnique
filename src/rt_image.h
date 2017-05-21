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
  RtImage(int, int);
  int getHeight() const;
  int getWidth() const;
  friend std::ostream &operator<<(std::ostream &, const RtImage &);
  std::vector<std::vector<RtColor>>& getImage();

private:
  int height_, width_;
  std::vector<std::vector<RtColor>> image_;
};

#endif /* RT_IMAGE_H */