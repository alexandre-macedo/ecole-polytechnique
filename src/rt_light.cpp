#include <stdexcept>

#include "rt_light.h"

// Make standard contructor and destructor later if you have a bug 
// I did not do one here just to test

RtLight::RtLight(){};
RtLight::~RtLight(){};

RtLight::RtLight(double x, double y, double z, int r, int g, int b) : point_{x, y, z}, color_{r, g, b}
{
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
        throw std::runtime_error("Invalid color.");
}

void RtLight::print(){
    std::cout << "Light with origin in (" << point_.x <<", " << point_.y << ", " << point_.z << ") and color [" << color_.r << ", " << color_.g << ", " << color_.b << "]" << std::endl;
}

std::ostream &operator<<(std::ostream &s, const RtLight &li){
    s << "[(" << li.point_.x << ", " << li.point_.y << ", " << li.point_.z << "), ["<< li.color_.r << ", " << li.color_.g << ", " << li.color_.b << "]]";
    return s;
};
