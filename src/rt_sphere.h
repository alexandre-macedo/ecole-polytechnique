#pragma once

class RtSphere{
     private:
        struct Color {int red, green, blue;};
        Color color;
        double radius;
        struct Center{double x, y, z;};
        Center center;
    public:
        //RtSphere();
        RtSphere(double, double, double, double, int, int, int);
        void print();
   
};