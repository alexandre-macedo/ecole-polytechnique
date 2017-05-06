#pragma once
#ifndef RT_RAY_H
#define RT_RAY_H

#include "rt_vector.h"

class RtRay{
    public:
        struct point_struct{
                double x, y, z;
            };
        RtRay(double, double, double, RtVector&);
        RtRay();
        ~RtRay();
        void print();

    private: 
        point_struct point; 
        RtVector direction;    
};

#endif /* RT_RAY_H */