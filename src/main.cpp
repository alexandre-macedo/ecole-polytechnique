#include "rt_ray.h"

int main(){
    RtVector dir(1.0, 1.0, 1.0);
  //  dir.print();
    RtRay teste(2.0, 2.0, 2.0, dir); 
    teste.print();
}