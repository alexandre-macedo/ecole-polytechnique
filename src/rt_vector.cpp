#include <cmath>

#include "rt_vector.h"

RtVector::RtVector() : x_(0), y_(0), z_(0) {}

RtVector::~RtVector() {}

RtVector::RtVector(double x, double y, double z) : x_(x), y_(y), z_(z) {}

void RtVector::print()
{
    std::cout << "Vector with coordinates [" << this->x_ << ", " << this->y_ << ", " << this->z_ << "]" << std::endl;
}

double RtVector::norm2()
{
    return sqrt(this->x_ * this->x_ + this->y_ * this->y_ + this->z_ * this->z_);
}

RtVector RtVector::operator+(const RtVector &v)
{
    RtVector vector;
    vector.x_ = this->x_ + v.x_;
    vector.y_ = this->y_ + v.y_;
    vector.z_ = this->z_ + v.z_;
    return vector;
}

RtVector RtVector::operator-()
{
    RtVector vector;
    vector.x_ = -this->x_;
    vector.y_ = -this->y_;
    vector.z_ = -this->z_;
    return vector;
}

RtVector RtVector::operator-(const RtVector &v)
{
    RtVector vector;
    vector.x_ = this->x_ - v.x_;
    vector.y_ = this->y_ - v.y_;
    vector.z_ = this->z_ - v.z_;
    return vector;
}

RtVector RtVector::operator*(const double &d)
{
    RtVector vector;
    vector.x_ = this->x_ * d;
    vector.y_ = this->y_ * d;
    vector.z_ = this->z_ * d;
    return vector;
}

RtVector RtVector::operator*(const RtVector &v)
{
    RtVector vector;
    vector.x_ = this->x_ * v.x_;
    vector.y_ = this->y_ * v.y_;
    vector.z_ = this->z_ * v.z_;
    return vector;
}

RtVector &RtVector::operator+=(const RtVector &v)
{
    *this = *this + v;
    return *this;
}

RtVector &RtVector::operator-=(const RtVector &v)
{
    *this = *this - v;
    return *this;
}

RtVector &RtVector::operator*=(const RtVector &v)
{
    *this = *this * v;
    return *this;
}

RtVector &RtVector::operator*=(const double &d)
{
    *this = *this * d;
    return *this;
}

std::ostream &operator<<(std::ostream &s, const RtVector &v)
{
    s << "[" << v.x_ << ", " << v.y_ << ", " << v.z_ << "]";
    return s;
}

RtVector operator*(const double d, const RtVector &v)
{
    RtVector vector = v;
    return vector * d;
}