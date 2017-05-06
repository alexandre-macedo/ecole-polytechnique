#include "rt_vector.h"
#include <iostream>

RtVector::RtVector(double x_, double y_, double z_): x(x_), y(y_), z(z_){}

RtVector::RtVector(): x(0), y(0), z(0){}

RtVector::~RtVector(){}

void RtVector::print(){
    std::cout << "[x, y, z] = [" << this->x << ", " << this->y << ", " << this->z << "]" << std::endl;
}

RtVector RtVector::operator+(const RtVector& v){
    RtVector vector;
    vector.x = this->x + v.x;
    vector.y = this->y + v.y;
    vector.z = this->z + v.z;
    return vector;
}

RtVector& RtVector::operator+=(const RtVector& v){
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
}

RtVector RtVector::operator*(const RtVector& v){
    RtVector vector;
    vector.x = this->x * v.x;
    vector.y = this->y * v.y;
    vector.z = this->z * v.z;
    return vector;
}

RtVector& RtVector::operator*=(const RtVector& v){
    this->x *= v.x;
    this->y *= v.y;
    this->z *= v.z;
    return *this;
}

RtVector RtVector::operator*(const double& d){
    RtVector vector;
    vector.x = this->x * d;
    vector.y = this->y * d;
    vector.z = this->z * d;
    return vector;
}