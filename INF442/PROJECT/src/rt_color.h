#pragma once
#ifndef RT_COLOR_H
#define RT_COLOR_H

#include <boost/mpi.hpp>

#include <iostream>

class RtColor {
public:
  RtColor();
  ~RtColor();
  RtColor(int r, int g, int b);
  friend std::ostream &operator<<(std::ostream &, const RtColor &);
  RtColor operator+(const RtColor &) const;
  RtColor operator*(const double)const;
  int getR() const;
  int getG() const;
  int getB() const;
  RtColor darker(double);
  void copyHere(const RtColor &);
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &r_;
    ar &g_;
    ar &b_;
  }

private:
  friend class boost::serialization::access;
  int r_, g_, b_;
};

#endif /* RT_COLOR_H */