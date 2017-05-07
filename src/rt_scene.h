#pragma once
#include "rt_sphere.h"
#include <iostream>
#include <vector>

class RtScene
{
  public:
    void add(RtSphere);
    RtSphere take_last();
    int size();
    void print();

  private:
    std::vector<RtSphere> list_of_spheres_;
};