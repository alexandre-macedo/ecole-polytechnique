#pragma once
#include "rt_vector.h"

class RtRay{
    public:
        RtRay();
        print();

    private: 
        struct point;
        RtVector direction;    
};