#include "rt_vector.h"

RtVector::RtVector(double x, double y, double z) : x_(x), y_(y), z_(z) {}

RtVector::RtVector() : x_(0), y_(0), z_(0) {}

RtVector::~RtVector() {}

void RtVector::print()
{
    std::cout << "Vector = [" << this->x_ << ", " << this->y_ << ", " << this->z_ << "]" << std::endl;
}

RtVector RtVector::operator+(const RtVector &v)
{
    RtVector vector;
    vector.x_ = this->x_ + v.x_;
    vector.y_ = this->y_ + v.y_;
    vector.z_ = this->z_ + v.z_;
    return vector;
}

RtVector &RtVector::operator+=(const RtVector &v)
{
    this->x_ += v.x_;
    this->y_ += v.y_;
    this->z_ += v.z_;
    return *this;
}

RtVector RtVector::operator*(const RtVector &v)
{
    RtVector vector;
    vector.x_ = this->x_ * v.x_;
    vector.y_ = this->y_ * v.y_;
    vector.z_ = this->z_ * v.z_;
    return vector;
}

RtVector &RtVector::operator*=(const RtVector &v)
{
    this->x_ *= v.x_;
    this->y_ *= v.y_;
    this->z_ *= v.z_;
    return *this;
}

RtVector RtVector::operator*(const double &d)
{
    RtVector vector;
    vector.x_ = this->x_ * d;
    vector.y_ = this->y_ * d;
    vector.z_ = this->z_ * d;
    return vector;
}

std::ostream &operator<<(std::ostream &s, const RtVector &v)
{
    s << "[" << v.x_ << ", " << v.y_ << ", " << v.z_ << "]";
    return s;
}