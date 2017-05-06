#include "rt_vector.h"
#include <iostream>

RtVector::RtVector(double x_, double y_, double z_): x(x_), y(y_), z(z_){}

RtVector::RtVector(): x(0), y(0), z(0){}

RtVector::~RtVector(){}

void RtVector::print(){
    std::cout << "[x, y, z] = [" << this->x << ", " << this->y << ", " << this->z << "]" << std::endl;
}

