#pragma once
#ifndef RT_VECTOR_H
#define RT_VECTOR_H

#include <iostream>

class RtVector
{
  public:
    RtVector();
    ~RtVector();
    RtVector(double, double, double);
    double norm2(); // normal norm, already with sqrt
    RtVector operator+(const RtVector &);
    RtVector operator-();
    RtVector operator-(const RtVector &);
    RtVector operator*(const RtVector &);
    RtVector operator*(const double &);
    RtVector &operator+=(const RtVector &);
    RtVector &operator-=(const RtVector &);
    RtVector &operator*=(const RtVector &);
    RtVector &operator*=(const double &d);
    friend std::ostream &operator<<(std::ostream &, const RtVector &);
    RtVector cross (const RtVector &);

  private:
    double x_, y_, z_;
};
RtVector operator*(const double, const RtVector &);

#endif /* RT_VECTOR_H */