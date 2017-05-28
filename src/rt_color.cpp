#include <stdexcept>

#include "rt_color.h"

RtColor::RtColor() : r_(0), g_(0), b_(0) {}

RtColor::~RtColor() {}

RtColor::RtColor(int r, int g, int b) : r_(r), g_(g), b_(b) {
  if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
    throw std::runtime_error("Invalid color.");
}

std::ostream &operator<<(std::ostream &s, const RtColor &cl) {
  s << "[" << cl.r_ << ", " << cl.g_ << ", " << cl.b_ << "]";
  return s;
}

RtColor RtColor::operator+(const RtColor &color) const {
  return RtColor((this->r_ + color.r_) > 255 ? 255 : (this->r_ + color.r_),
                 (this->g_ + color.g_) > 255 ? 255 : (this->g_ + color.g_),
                 (this->b_ + color.b_) > 255 ? 255 : (this->b_ + color.b_));
}

RtColor RtColor::operator*(const double multiplier) const {
  if (multiplier < 0 || multiplier > 1)
    throw std::runtime_error("Bad Color multiplier.");
  return RtColor((this->r_ * multiplier) > 255 ? 255 : (this->r_ * multiplier),
                 (this->g_ * multiplier) > 255 ? 255 : (this->g_ * multiplier),
                 (this->b_ * multiplier) > 255 ? 255 : (this->b_ * multiplier));
}

int RtColor::getR() const { return this->r_; }

int RtColor::getG() const { return this->g_; }

int RtColor::getB() const { return this->b_; }

RtColor RtColor::darker(double percentage) {
  if (percentage < 0 || percentage > 1)
    throw std::runtime_error("Invalid 'dark' percentage.");
  double comple = 1.0 - percentage;
  return RtColor(this->r_ *= comple, this->g_ *= comple, this->b_ *= comple);
}

void RtColor::copyHere(const RtColor &other) {
  r_ = other.getR();
  g_ = other.getG();
  b_ = other.getB();
}