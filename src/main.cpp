#include "rt_ray.h"

int main()
{
  RtVector dir(1.0, 2.0, 3.0);
  RtVector dir2(4.0, 5.0, 6.0);
  std::cout << dir << std::endl;
  std::cout << dir2 << std::endl;
  dir += dir2;
  dir.print();
  //  dir.print();
  RtRay ray(2.0, 2.0, 2.0, dir);
  std::cout << ray << std::endl;
  ray.print();
}