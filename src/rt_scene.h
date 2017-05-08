#pragma once
#ifndef RT_SCENE_H
#define RT_SCENE_H

#include <iostream>
#include <vector>

#include "rt_sphere.h"

class RtScene
{
  public:
    RtScene();
    ~RtScene();
    void add(RtSphere);
    RtSphere take_last();
    int size();
    RtSphere at_index(int);
    friend std::ostream &operator<<(std::ostream &, const RtScene &);
    
  private:
    std::vector<RtSphere> list_of_spheres_;
};

#endif /* RT_SCENE_H */