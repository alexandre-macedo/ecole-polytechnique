#include <stdexcept>

#include "rt_image.h"

RtImage::RtImage() {}
RtImage::~RtImage() {}
RtImage::RtImage(const int height, const int width)
    : height_(height), width_(width),
      image_(width, std::vector<RtColor>(height)) {
  if (height <= 0 || width <= 0)
    throw std::runtime_error("Invalid resolution.");
}

int RtImage::getHeight() const { return this->height_; }

int RtImage::getWidth() const { return this->width_; }

std::string RtImage::info() const {
  std::string s;
  s = "[" + std::to_string(this->height_) + ", " + std::to_string(this->width_) + "]";
  return s;
}

std::ostream &operator<<(std::ostream &s, const RtImage &im) {
  s << "[" << im.height_ << ", " << im.width_ << "] Image:" << std::endl;
  for (int j = 0; j < im.height_; ++j) {
    for (int i = 0; i < im.width_; ++i)
      s << im.image_[i][j] << ",";
    s << std::endl;
  }

  return s;
}

std::vector<std::vector<RtColor>> &RtImage::getImage() { return this->image_; }
