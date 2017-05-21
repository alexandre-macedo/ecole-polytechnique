// Methods / tools for the project

#include "rt_ray.h"
#include "rt_sphere.h"

namespace RtTools {
bool intersection(const RtSphere &, const RtRay &, RtVector &);
}
// We will fill the next line with the prototype of "bool intersection()"