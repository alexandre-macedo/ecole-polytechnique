#include <cmath>

#include "rt_vector.h"

RtVector::RtVector() : x_(0), y_(0), z_(0) {}

RtVector::~RtVector() {}

RtVector::RtVector(double x, double y, double z) : x_(x), y_(y), z_(z) {}

double RtVector::norm2() const {
  return sqrt(this->x_ * this->x_ + this->y_ * this->y_ + this->z_ * this->z_);
}

RtVector RtVector::operator+(const RtVector &v) const {
  RtVector vector;
  vector.x_ = this->x_ + v.x_;
  vector.y_ = this->y_ + v.y_;
  vector.z_ = this->z_ + v.z_;
  return vector;
}

RtVector RtVector::operator-() const {
  RtVector vector;
  vector.x_ = -this->x_;
  vector.y_ = -this->y_;
  vector.z_ = -this->z_;
  return vector;
}

RtVector RtVector::operator-(const RtVector &v) const {
  RtVector vector;
  vector.x_ = this->x_ - v.x_;
  vector.y_ = this->y_ - v.y_;
  vector.z_ = this->z_ - v.z_;
  return vector;
}

RtVector RtVector::operator*(const double &d) const {
  RtVector vector;
  vector.x_ = this->x_ * d;
  vector.y_ = this->y_ * d;
  vector.z_ = this->z_ * d;
  return vector;
}

<<<<<<< HEAD
double RtVector::operator*(const RtVector &v) const {
  return this->x_ * v.x_ + this->y_ * v.y_ + this->z_ * v.z_;
=======
double RtVector::operator*(const RtVector &v) {
  double dotProduct = 0.;
  dotProduct += this->x_ * v.x_;
  dotProduct += this->y_ * v.y_;
  dotProduct += this->z_ * v.z_;
  return dotProduct;
>>>>>>> 7384cc5b8b087da3847731904b03245a19a35ee8
}

RtVector &RtVector::operator+=(const RtVector &v) {
  *this = *this + v;
  return *this;
}

RtVector &RtVector::operator-=(const RtVector &v) {
  *this = *this - v;
  return *this;
}

RtVector &RtVector::operator*=(const double &d) {
  *this = *this * d;
  return *this;
}

std::ostream &operator<<(std::ostream &s, const RtVector &v) {
  s << "(" << v.x_ << ", " << v.y_ << ", " << v.z_ << ")";
  return s;
}

RtVector operator*(const double d, const RtVector &v) {
  RtVector vector = v;
  return vector * d;
}

RtVector RtVector::cross(const RtVector &v) {
  return RtVector(this->y_ * v.z_ - v.y_ * this->z_,
                  -this->x_ * v.z_ + v.x_ * this->z_,
                  this->x_ * v.y_ - v.x_ * this->y_);
}

RtVector RtVector::unit() const { return *this * (1.0 / this->norm2()); }