#pragma once
#ifndef RT_TOOLS_H
#define RT_TOOLS_H

#include "rt_ray.h"
#include "rt_vector.h"
#include "rt_light.h"
#include "rt_sphere.h"

namespace RtTools{
    // if there is intersection will return true and put the point of intersection in "i" 
    bool intersection(RtSphere &sph, RtRay &r, RtVector &i);
    
    // "viewer" is the vector from the camera pointing towards the point 
    RtColor colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer, RtLight &light);
}

#endif /* RT_TOOLS_H */