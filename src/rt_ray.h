#pragma once
#ifndef RT_RAY_H
#define RT_RAY_H

#include "rt_vector.h"

class RtRay{
    public:
        struct point_struct{
                double x;
                double y;
                double z;
            };
        RtRay(double, double, double, RtVector&);
        RtRay();
<<<<<<< HEAD
=======
        ~RtRay();
>>>>>>> 920808f57ff7efac4fd06edc8626056eec4beb82
        void print();

    private: 
        point_struct point; 
        RtVector direction;    
};

#endif /* RT_RAY_H */