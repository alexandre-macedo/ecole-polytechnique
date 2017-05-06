#pragma once
#ifndef RT_VECTOR_H
#define RT_VECTOR_H

class RtVector{
    
public:
    RtVector(double , double, double);
    RtVector();
    ~RtVector();
    void print();
    RtVector operator+(const RtVector& v);
    RtVector& operator+=(const RtVector& v);
    RtVector operator*(const RtVector& v);
    RtVector& operator*=(const RtVector& v);
    RtVector operator*(const double& d);

private:
    double x,y,z;

};

#endif /* RT_VECTOR_H */