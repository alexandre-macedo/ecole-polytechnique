#include <stdexcept>

#include "rt_color.h"

RtColor::RtColor() {}

RtColor::~RtColor() {}

RtColor::RtColor(int r, int g, int b) : r_(r), g_(g), b_(b) {
  if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
    throw std::runtime_error("Invalid color.");
}

std::ostream &operator<<(std::ostream &s, const RtColor &cl) {
  s << "[" << cl.r_ << ", " << cl.g_ << ", " << cl.b_ << "]";
  return s;
}

int RtColor::getR() const { return this->r_; }

int RtColor::getG() const { return this->g_; }

int RtColor::getB() const { return this->b_; }