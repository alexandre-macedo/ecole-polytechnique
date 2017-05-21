#include <stdexcept>

#include "rt_light.h"

// Make standard contructor and destructor later if you have a bug 
// I did not do one here just to test

RtLight::RtLight(){};
RtLight::~RtLight(){};

RtLight::RtLight(RtVector pt, int r, int g, int b) : point_(pt), color_{r, g, b}
{
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
        throw std::runtime_error("Invalid color.");
}

std::ostream &operator<<(std::ostream &s, const RtLight &li){
    s << "[" << li.point_ << ", ["<< li.color_.r << ", " << li.color_.g << ", " << li.color_.b << "]]";
    return s;
};

RtVector RtLight::getPoint(){
    return point_;    
}
