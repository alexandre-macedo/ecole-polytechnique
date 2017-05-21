#pragma once
#ifndef RT_LIGHT_H
#define RT_LIGHT_H

#include <iostream>

#include "rt_vector.h"
#include "rt_color.h"

class RtLight{
    public:
        RtLight();
        ~RtLight();
        RtLight(RtVector, RtColor);
        friend std::ostream &operator<<(std::ostream &, const RtLight &);
    private:
        RtVector point_;
        RtColor color_;
};



#endif /* RT_LIGHT_H */