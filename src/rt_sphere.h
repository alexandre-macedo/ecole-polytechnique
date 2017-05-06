#pragma once

class RtSphere{
     private:
        enum Color {red, green, blue};
        Color color;
        double radius;
        struct Center{
            double x, y, z; 
        };
        Center center;
    public:
        RtSphere();
        RtSphere(double, double, double, double, Color);
        void print();
   
};