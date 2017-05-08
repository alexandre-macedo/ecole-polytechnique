#include "rt_ray.h"
#include "rt_sphere.h"
#include "rt_scene.h"
#include "rt_light.h"
#include "rt_camera.h"


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
  // dir.print();n

  // Testing Ray:
  std::cout << "########## Testing Ray ##########" << std::endl;
  RtVector pt (1.5, 2.0, 2.0);
  RtRay ray(pt, dir);
  std::cout << ray << std::endl;

  // Testing Spheres:
  std::cout << "########## Testing Spheres ##########" << std::endl;
  RtSphere sph(RtVector(1., 2., 3.), 1., 0, 0, 0);
  std::cout << sph << std::endl;

  // Testing Scenes:
  std::cout << "########## Testing Scenes ##########" << std::endl;
  RtSphere sph2(RtVector(0.,0.,0.), 3., 37, 73, 254);
  RtScene sc;
  sc.add(sph);
  sc.add(sph2);
  //sc.take_last();
  std::cout << "Taking the second sphere of the scene: " << std::endl;
  std::cout << sc.at_index(1) << std::endl;

  // Testing Light:
  std::cout << "########## Testing Ligth ##########" << std::endl;
  RtLight li (RtVector(11., 11., 11.), 0, 0, 0);
  std::cout << li << std::endl;

   // Testing Camera
   std::cout << "########## Testing Camera ##########" << std::endl;
   RtCamera cam(dir, dir2, dir, 10.0, 20.0);
   std::cout << cam << std::endl;

   // Testing Vectorial Product
   std::cout << "########## Testing Vectorial Product ##########" << std::endl;
   RtVector v1 (1., 0., 0.);
   RtVector v2 (0., 1., 0.);
   std::cout << v1.cross(v2) << std::endl;

   // Testing Ray-sphere intersection
   std::cout << "########## Testing Ray-sphere intersection ##########" << std::endl;
   RtRay r (RtVector (-4., 0., 0.), RtVector(1., 0., 0.)); 
   RtSphere o (RtVector(0., 0., 0.), 1., 0, 0, 0);
   RtVector in;
   std::cout << "Has to be true: " << o.intersectionWith(r, in) << std::endl;
   std::cout << "Insection has to be (-1, 0, 0): " << in << std::endl;
}



