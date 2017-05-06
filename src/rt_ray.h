#pragma once
#include "rt_vector.h"

class RtRay{
    public:
        RtRay();
        void print();

    private: 
        struct point;
        RtVector direction;    
};