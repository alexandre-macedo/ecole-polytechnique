#pragma once
#ifndef RT_LIGHT_H
#define RT_LIGHT_H

#include <iostream>

class RtLight{
    public:
        RtLight();
        ~RtLight();
        RtLight(double, double, double, int, int, int);
        void print();
        friend std::ostream &operator<<(std::ostream &, const RtLight &);
    private:
        struct{double x, y, z;} point_;
        struct{int r, g, b;} color_;
};



#endif /* RT_LIGHT_H */