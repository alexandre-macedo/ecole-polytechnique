#include "rt_ray.h"
#include "rt_sphere.h"
#include "rt_scene.h"
<<<<<<< HEAD
#include "rt_light.h"
=======
#include "rt_camera.h"
>>>>>>> 80640db1eef801d9624d3126cbc9c40bc291cf33

int main()
{
  // Testing Vector:
  std::cout << "########## Testing Vector ##########" << std::endl;
  RtVector dir(1.0, 2.0, 3.0);
  RtVector dir2(4.0, 5.0, 6.0);
  std::cout << dir << std::endl;
  std::cout << dir2 << std::endl;
  std::cout << dir + dir2 << std::endl;
  std::cout << -dir2 << std::endl;
  std::cout << dir - dir2 << std::endl;
  std::cout << dir.norm2() << std::endl;
  std::cout << dir * 2.0 << std::endl;
  std::cout << 3.0 * dir << std::endl;
  dir += dir2;
  dir *= 2.0;
  dir.print();
  // dir.print();n

  // Testing Ray:
  std::cout << "########## Testing Ray ##########" << std::endl;
  RtRay ray(2.0, 2.0, 2.0, dir);
  std::cout << ray << std::endl;
  ray.print();

  // Testing Spheres:
  std::cout << "########## Testing Spheres ##########" << std::endl;
  RtSphere sph(0., 0., 0., 1., 0, 0, 0);
  std::cout << sph << std::endl;
  sph.print();

  // Testing Scenes:
  std::cout << "########## Testing Scenes ##########" << std::endl;
  RtSphere sph2(0., 0., 0., 3., 37, 73, 254);
  RtScene sc;
  sc.add(sph);
  sc.add(sph2);
  //sc.take_last();
  sc.print();
  std::cout << "Taking the second sphere of the scene: " << std::endl;
  sc.at_index(1).print();

  // Testing Light:
  std::cout << "########## Testing Ligth ##########" << std::endl;
  RtLight li (10., 10., 10., 0, 0, 0);
  std::cout << li << std::endl;

  std::cout << sc << std::endl;

  // Testing Camera
   std::cout << "########## Testing Camera ##########" << std::endl;
   RtCamera cam(dir, dir2, dir, 10.0, 20.0);
   std::cout << cam << std::endl;

}