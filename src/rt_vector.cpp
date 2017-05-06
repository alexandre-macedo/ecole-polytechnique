#include "rt_vector.h"

RtVector::RtVector(double x_, double y_, double z_): x(x_), y(y_), z(z_){}

void print(){
    std::cout << "[x, y, z] = [" << this->x << ", " << this->y << ", " << this->z << "]";
}

