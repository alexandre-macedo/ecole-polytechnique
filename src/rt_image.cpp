#include <stdexcept>

#include "rt_image.h"

RtImage::RtImage() {}
RtImage::~RtImage() {}
RtImage::RtImage(int height, int width)
    : height_(height), width_(width),
      image_(height, std::vector<RtColor>(width)) {
  if (height <= 0 || width <= 0)
    throw std::runtime_error("Invalid resolution.");
}
