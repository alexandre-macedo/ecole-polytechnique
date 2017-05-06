#pragma once
#ifndef RT_VECTOR_H
#define RT_VECTOR_H

class RtVector{
    
public:
    RtVector(double x_, double y_, double z_);
    ~RtVector();
    void print();

private:
    double x,y,z;

};

#endif /* RT_VECTOR_H */