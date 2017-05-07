#pragma once
#ifndef RT_VECTOR_H
#define RT_VECTOR_H

#include <iostream>

class RtVector
{
  public:
    RtVector(double, double, double);
    RtVector();
    ~RtVector();
    void print();
    RtVector operator+(const RtVector &);
    RtVector &operator+=(const RtVector &);
    RtVector operator*(const RtVector &);
    RtVector &operator*=(const RtVector &);
    RtVector operator*(const double &);
    friend std::ostream &operator<<(std::ostream &, const RtVector &);

  private:
    double x_, y_, z_;
};

#endif /* RT_VECTOR_H */