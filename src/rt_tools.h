#pragma once
#ifndef RT_TOOLS_H
#define RT_TOOLS_H

#include <string>

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
enum class imageType { SIMPLE , WITHSHADOWS, WITHREFLECTION};

bool intersection(const RtSphere &, const RtRay &, RtVector &);

bool intersection(const RtScene &, const RtRay &, RtSphere &, RtVector &);

// "viewer" is the vector from the camera pointing towards the point
RtColor colorOfPoint(const RtVector &, const RtSphere &, const RtVector &,
                     const RtLight &);

// void generateSimpleImage(const RtScene &, const RtCamera &, const RtLight &,
//                          RtImage &);

// void generateImageWithShadows(const RtScene &, const RtCamera &,
//                               const RtLight &, RtImage &);

void generateImage(const RtScene &scene, const RtCamera &camera,
                   const RtLight &light, RtImage &rtimage, imageType);

void convertToOpenCV(const RtImage &, cv::Mat &);

void printCVImage(const cv::Mat &);

void saveCVImage(const cv::Mat &, const std::string &);
}

#endif /* RT_TOOLS_H */