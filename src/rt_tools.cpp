// Methods / tools for the project

#include <cmath>

#include "rt_tools.h"

namespace RtTools
{
bool intersection(const RtSphere &sph, const RtRay &r, RtVector &i)
{
    RtVector center = sph.getCenter();
    double radius = sph.getRadius();
    RtVector dir = r.getDirection();
    dir = dir.unit(); // norm == 1 now
    RtVector p = r.getOrigin();
    double d = ((dir.cross(center - p)).norm2());

    if (d > radius)
    {
        return false;
    }

    double hip = (center - p).norm2();
    double cateto = sqrt(hip * hip - d * d);
    double correction = sqrt(radius * radius - d * d);
    // Updating intersection point
    //std::cout << "p: " << p << " dir: " << dir << " cat: " << cateto << " dist: " << d << std::endl;;
    i = p + dir * (cateto - correction);

    return true;
}
}