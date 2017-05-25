#pragma once
#ifndef RT_TOOLS_H
#define RT_TOOLS_H

#include "rt_camera.h"
#include "rt_image.h"
#include "rt_light.h"
#include "rt_ray.h"
#include "rt_scene.h"
#include "rt_sphere.h"
#include "rt_vector.h"

namespace RtTools {
bool intersection(RtSphere &, RtRay &, RtVector &);

// "viewer" is the vector from the camera pointing towards the point
RtColor colorOfPoint(RtVector &, RtSphere &, RtVector &, const RtLight &);

void generateImage(const RtScene &, const RtCamera &, const RtLight &,
                   RtImage &);
}

#endif /* RT_TOOLS_H */