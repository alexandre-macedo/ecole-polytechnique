#pragma once
#ifndef RT_VECTOR_H
#define RT_VECTOR_H

#include <iostream>

class RtVector
{
  public:
    RtVector();
    RtVector(double, double, double);
    ~RtVector();
    void print();
    double norm2();
    RtVector operator+(const RtVector &);
    RtVector operator-();
    RtVector operator-(const RtVector &);
    RtVector operator*(const RtVector &);
    RtVector operator*(const double &);
    RtVector &operator+=(const RtVector &);
    RtVector &operator-=(const RtVector &);
    RtVector &operator*=(const RtVector &);
    RtVector &operator*=(const double &d);
    friend RtVector operator*(const double, const RtVector &);
    friend std::ostream &operator<<(std::ostream &, const RtVector &);

  private:
    double x_, y_, z_;
};

#endif /* RT_VECTOR_H */