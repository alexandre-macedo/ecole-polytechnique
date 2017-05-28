#pragma once
#ifndef RT_COLOR_H
#define RT_COLOR_H

#include <iostream>

class RtColor {
public:
  RtColor();
  ~RtColor();
  RtColor(int r, int g, int b);
  friend std::ostream &operator<<(std::ostream &, const RtColor &);
  RtColor operator+(const RtColor&) const;
  RtColor operator*(const double) const;
  int getR() const;
  int getG() const;
  int getB() const;
  RtColor darker(double);
  void copyHere(const RtColor &);

private:
  int r_, g_, b_;
};

#endif /* RT_COLOR_H */