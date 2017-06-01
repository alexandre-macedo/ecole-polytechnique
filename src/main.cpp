#include <fstream>
#include <iostream>
#include <string>

#include <boost/mpi.hpp>
#include <opencv2/core/core.hpp>

#include "rt_camera.h"
#include "rt_color.h"
#include "rt_light.h"
#include "rt_ray.h"
#include "rt_scene.h"
#include "rt_sphere.h"
#include "rt_tools.h"

int main() {
  
  // // Cor
  // RtColor cor(0, 0, 0);

  //   // Testing Vector:
  //   std::cout << "########## Testing Vector ##########" << std::endl;
  //   RtVector dir(1.0, 2.0, 3.0);
  //   RtVector dir2(4.0, 5.0, 6.0);
  //   dir += dir2;
  //   dir *= 2.0;
  //   std::cout << dir.unit() << std::endl;
  //   // dir.print();n

  //   // Testing Ray:
  //   std::cout << "########## Testing Ray ##########" << std::endl;
  //   RtVector pt(1.5, 2.0, 2.0);
  //   RtRay ray(pt, dir);
  //   std::cout << ray << std::endl;

  //   // Testing Spheres:
  //   std::cout << "########## Testing Spheres ##########" << std::endl;
  //   RtSphere sph(RtVector(1., 2., 3.), 1., cor);
  //   std::cout << sph << std::endl;

  //   // Testing Scenes:
  //   std::cout << "########## Testing Scenes ##########" << std::endl;
  //   RtSphere sph2(RtVector(0., 0., 0.), 3., cor);
  //   RtScene sc;
  //   sc.add(sph);
  //   sc.add(sph2);
  //   // sc.take_last();
  //   std::cout << "Taking the second sphere of the scene: " << std::endl;
  //   std::cout << sc.at_index(1) << std::endl;

  //   // Testing Light:
  //   std::cout << "########## Testing Ligth ##########" << std::endl;
  //   RtLight li(RtVector(11., 11., 11.), cor);
  //   std::cout << li << std::endl;

  //   // Testing Camera
  //   std::cout << "########## Testing Camera ##########" << std::endl;
  //   RtCamera cam(RtVector(0, 0, 0), RtVector(0, 1, 0), RtVector(1, 1, 0),
  //   10.0,
  //                20.0);
  //   std::cout << cam << std::endl;

  //   // Testing Vectorial Product
  //   std::cout << "########## Testing Vectorial Product ##########" <<
  //   std::endl;
  //   RtVector v1(1., 0., 0.);
  //   RtVector v2(0., 1., 0.);
  //   std::cout << v1.cross(v2) << std::endl;

  //   // Testing Ray-sphere intersection
  //   std::cout << "########## Testing Ray-sphere intersection ##########"
  //             << std::endl;

  //   RtRay r(RtVector(-4., 0., 0.), RtVector(1., 0., 0.));
  //   RtSphere o(RtVector(0., 0., 0.), 1., cor);
  //   RtVector in;
  //   std::cout << "Has to be true: " << RtTools::intersection(o, r, in)
  //             << std::endl;
  //   std::cout << "Insection has to be (-1, 0, 0): " << in << std::endl;

  //   // Testing colorOfPoint from tools
  //   std::cout << "########## Testing colorOfPoint from tools ##########"
  //             << std::endl;
  //   RtColor c(100, 100, 100);
  //   RtSphere esfera(RtVector(3., 0., 0.), 1., c);
  //   RtVector ponto(2., 0., 0.);
  //   RtVector doOlhoAoPonto(1., 0., 0.); // olho em (-3.,0., 0.)
  //   RtVector ptLuz(-2., 0., 0.);
  //   RtLight luz(ptLuz, c);
  //   std::cout << "Color of point should be [67, 67, 67] and it is: "
  //             << RtTools::colorOfPoint(ponto, esfera, doOlhoAoPonto, luz)
  //             << std::endl;

  // Testing sort in scene
  // std::cout << "########## Testing sort in scene ##########" << std::endl;
  // RtSphere esfera1(RtVector(3., 0., 0.), .1, cor);
  // RtSphere esfera2(RtVector(1., 0., 0.), .1, cor);
  // RtSphere esfera3(RtVector(2.1, 0., 0.), .1, cor);
  // RtScene cena;
  // cena.add(esfera1);
  // cena.add(esfera2);
  // cena.add(esfera3);
  // RtVector ref(0., 0., 0.);
  // cena.sort(ref);
  // std::cout << "Should be in the increasing distance from the origin: "
  //           << std::endl;
  // for (int i = 0; i < cena.size(); i++) {
  //   std::cout << "Position " << i << " : " << cena.at_index(i) << std::endl;
  // }

  // // Testing Image Creation
  
  std::cout << "Testing image Creation" << std::endl;


  RtCamera camera_image_test(RtVector(0, 1, 1), RtVector(1, 1, 1),
                             RtVector(0, 1, 0), 30, 20);
  std::cout << "Camera:" << camera_image_test << std::endl;

  RtSphere sphere_image_test(RtVector(4, 0, 0), 2.0, RtColor(51, 255, 51), 0.0);
  std::cout << "Sphere:" << sphere_image_test << std::endl;


  RtSphere sphere_image_test2(RtVector(4, 6, 0), 2.0, RtColor(51, 255, 255),
                              0.5);
  std::cout << "Sphere:" << sphere_image_test2 << std::endl;

  RtSphere sphere_image_test3(RtVector(4, 4, 8), 2.0, RtColor(51, 255, 255),
                              0.3);
  std::cout << "Sphere:" << sphere_image_test3 << std::endl;

  RtScene scene_image_test;
  scene_image_test.add(sphere_image_test);
  scene_image_test.add(sphere_image_test2);
  scene_image_test.add(sphere_image_test3);
  std::cout << "Scene:" << scene_image_test << std::endl;

  RtLight light_image_test(RtVector(-5, 0, 0), RtColor(0, 0, 0));
  std::cout << "Light:" << light_image_test << std::endl;

  RtImage image_image_test(900, 600);
  std::cout << "Image:" << image_image_test.info() << std::endl;
  // RtImage image_image_test2(800, 800);
  // std::cout << "Image:" << image_image_test.info() << std::endl;


  // RtScene new_scene;

  // RtTools::newScene(scene_image_test, camera_image_test, new_scene);
  // std::cout << "Scene2:" << new_scene << std::endl;
  namespace mpi = boost::mpi;
  mpi::environment env;
  mpi::communicator world;


  RtTools::MPIenerateImage(scene_image_test, camera_image_test,
                            light_image_test, image_image_test,
                            RtTools::Shadows::ON, RtTools::Reflection::ON);
  if(world.rank()==0){
    cv::Mat output;
    RtTools::convertToOpenCV(image_image_test, output);
    RtTools::saveCVImage(output, "test1");
    RtTools::printCVImage(output);
  }

  // cv::Mat output2;
  // int i = 0;
  // while (1) {
  //   light_image_test = RtLight(
  //       RtVector(10 * cos(i / 20.0), 0, 10 * sin(i / 20.0)), RtColor(0, 0, 0));
  //   RtTools::MPIgenerateImage(scene_image_test, camera_image_test,
  //                             light_image_test, image_image_test2,
  //                             RtTools::Shadows::ON, RtTools::Reflection::ON);
  //   if (world.rank() == 0) {
  //     RtTools::convertToOpenCV(image_image_test2, output2);
  //     cv::imshow("Teste", output2);
  //     cv::waitKey(30);
  //     i++;
  //   }
  // }

  // RtRay raio(RtVector(4, 5, 0), RtVector(0, 1, 0));
  // std::cout << "Ray:" << raio << std::endl;

  // RtVector result;

  // std::cout << "Intersection ? "
  //           << RtTools::intersection(sphere_image_test, raio, result)
  //           << std::endl;
  // std::cout << "Intersection:" << result << std::endl;

  // namespace mpi = boost::mpi;
  // std::cout << "Testing serialization" << std::endl;
  // RtColor color(20, 20, 30);
  // mpi::environment env;
  // mpi::communicator world;
  // if (world.rank() == 0) {
  //   RtColor color(20, 20, 30);
  //   std::cout << "I am process " << world.rank() << " of " << world.size()
  //             << ". Sending information." << std::endl;
  //   std::cout << color << std::endl;
  //   world.send(1, 0, color);
  // } else {
  //   RtColor color2;
  //   world.recv(0, 0, color2);
  //   std::cout << "I am process " << world.rank() << " of " << world.size()
  //             << "." << std::endl;
  //   std::cout << color2 << std::endl;
  // }
}
