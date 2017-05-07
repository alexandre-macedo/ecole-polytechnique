#pragma once
#include "rt_sphere.h"
#include <iostream>
#include <vector>
using namespace std;

class RtScene{
    private:
        vector<RtSphere> list_of_spheres;

    public:
        void add(RtSphere);
        RtSphere take_last();
        int size();
        void print();

};