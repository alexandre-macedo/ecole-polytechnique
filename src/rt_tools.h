#pragma once
#ifndef RT_TOOLS_H
#define RT_TOOLS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rt_camera.h"
#include "rt_image.h"
#include "rt_light.h"
#include "rt_ray.h"
#include "rt_scene.h"
#include "rt_sphere.h"
#include "rt_vector.h"

namespace RtTools {
bool intersection(const RtSphere &, const RtRay &, RtVector &);

bool intersection(const RtScene &, const RtRay &, RtSphere &, RtVector &);

// "viewer" is the vector from the camera pointing towards the point
RtColor colorOfPoint(RtVector &, RtSphere &, RtVector &, const RtLight &);

void generateImage(const RtScene &, const RtCamera &, const RtLight &,
                   RtImage &);

void convertToOpenCV(RtImage &, cv::Mat &);
    // if there is intersection will return true and put the point of intersection in "i" 
    bool intersection(RtSphere &sph, RtRay &r, RtVector &i);
    
    // "viewer" is the vector from the camera pointing towards the point 
    RtColor colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer, RtLight &light);

void printCVImage(const cv::Mat &);
}

#endif /* RT_TOOLS_H */