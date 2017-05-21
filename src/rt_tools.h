// Methods / tools for the project

#include "rt_ray.h"
#include "rt_vector.h"
#include "rt_light.h"
#include "rt_sphere.h"

namespace RtTools{
    bool intersection(RtSphere &sph, RtRay &r, RtVector &i);
    
    // "viewer" is the vector from the camera pointing towards the point 
    RtColor colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer, RtLight &light);
}

