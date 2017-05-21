// Methods / tools for the project

#include <cmath>

#include "rt_tools.h"

namespace RtTools{
    bool intersection(RtSphere &sph, RtRay &r, RtVector &i){
        RtVector center = sph.getCenter(); double radius = sph.getRadius();
        RtVector dir = r.getDirection();
        dir = dir.unit(); // norm == 1 now
        RtVector p = r.getOrigin();
        double d = ((dir.cross(center-p)).norm2());
        
        if(d > radius){
            return false;
        }
        
        double hip = (center-p).norm2();
        double cateto = sqrt(hip*hip - d*d);
        double correction = sqrt(radius*radius - d*d);
        // Updating intersection point 
        //std::cout << "p: " << p << " dir: " << dir << " cat: " << cateto << " dist: " << d << std::endl;;
        i = p + dir*(cateto-correction);

        return true;
    }

    // "viewer" is the vector from the camera pointing towards the point 
    // using Phong Reflection Model - https://en.wikipedia.org/wiki/Phong_reflection_model
    int colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer, RtLight &light){
        RtVector V = ((-1)*viewer).unit(); // viewer
        RtVector N = (pt-sph.getCenter()).unit(); // normal
        RtVector L = (light.getPoint() - pt).unit(); // light

        return 1000;
    }
}