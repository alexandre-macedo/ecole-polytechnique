#pragma once
#ifndef RT_LIGHT_H
#define RT_LIGHT_H

#include <iostream>

#include "rt_vector.h"

class RtLight{
    public:
        RtLight();
        ~RtLight();
        RtLight(RtVector, int, int, int);
        friend std::ostream &operator<<(std::ostream &, const RtLight &);
    private:
        RtVector point_;
        struct{int r, g, b;} color_;
};



#endif /* RT_LIGHT_H */