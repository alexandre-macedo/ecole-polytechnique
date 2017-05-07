#pragma once
#ifndef RT_SCENE_H
#define RT_SCENE_H

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
    RtSphere at_index(int);

  private:
    std::vector<RtSphere> list_of_spheres_;
};

#endif /* RT_SCENE_H */