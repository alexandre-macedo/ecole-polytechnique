#pragma once
#ifndef RT_SPHERE_H
#define RT_SPHERE_H

#include <iostream>

class RtSphere
{
  public:
    RtSphere();
    ~RtSphere();
    RtSphere(double, double, double, double, int, int, int);
    void print();
    friend std::ostream &operator<<(std::ostream &, const RtSphere &);

  private:
    struct { double x, y, z;} center_;
    struct { int r, g, b;} color_;
    double radius_;
};

#endif /* RT_SPHERE_H */