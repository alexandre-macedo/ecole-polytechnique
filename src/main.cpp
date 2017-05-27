#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include "rt_camera.h"
#include "rt_color.h"
#include "rt_light.h"
#include "rt_ray.h"
#include "rt_scene.h"
#include "rt_sphere.h"
#include "rt_tools.h"

int main() {
  // // Testing Vector:
  // std::cout << "########## Testing Vector ##########" << std::endl;
  // RtVector dir(1.0, 2.0, 3.0);
  // RtVector dir2(4.0, 5.0, 6.0);
  // dir += dir2;
  // dir *= 2.0;
  // RtVector dir3;
  // dir3 = 4 * dir;
  // std::cout << dir.unit() << std::endl;
  // // dir.print();n

  // // Cor
  // RtColor cor(0, 0, 0);

  // // Testing Ray:
  // std::cout << "########## Testing Ray ##########" << std::endl;
  // RtVector pt(1.5, 2.0, 2.0);
  // RtRay ray(pt, dir);
  // std::cout << ray << std::endl;

  // // Testing Spheres:
  // std::cout << "########## Testing Spheres ##########" << std::endl;
  // RtSphere sph(RtVector(1., 2., 3.), 1., cor);
  // std::cout << sph << std::endl;

  // // Testing Scenes:
  // std::cout << "########## Testing Scenes ##########" << std::endl;
  // RtSphere sph2(RtVector(0., 0., 0.), 3., cor);
  // RtScene sc;
  // sc.add(sph);
  // sc.add(sph2);
  // // sc.take_last();
  // std::cout << "Taking the second sphere of the scene: " << std::endl;
  // std::cout << sc.at_index(1) << std::endl;

  // // Testing Light:
  // std::cout << "########## Testing Ligth ##########" << std::endl;
  // RtLight li(RtVector(11., 11., 11.), cor);
  // std::cout << li << std::endl;

  // // Testing Camera
  // std::cout << "########## Testing Camera ##########" << std::endl;
  // RtCamera cam(RtVector(0, 0, 0), RtVector(0, 1, 0), RtVector(1, 1, 0), 10.0,
  //              20.0);
  // std::cout << cam << std::endl;

  // // Testing Vectorial Product
  // std::cout << "########## Testing Vectorial Product ##########" <<
  // std::endl;
  // RtVector v1(1., 0., 0.);
  // RtVector v2(0., 1., 0.);
  // std::cout << v1.cross(v2) << std::endl;

  // // Testing Ray-sphere intersection
  // std::cout << "########## Testing Ray-sphere intersection ##########"
  //           << std::endl;

  // RtRay r(RtVector(-4., 0., 0.), RtVector(1., 0., 0.));
  // RtSphere o(RtVector(0., 0., 0.), 1., cor);
  // RtVector in;
  // std::cout << "Has to be true: " << RtTools::intersection(o, r, in)
  //           << std::endl;
  // std::cout << "Insection has to be (-1, 0, 0): " << in << std::endl;

  // // Test Image
  // std::cout << "########## Testing Image ##########" << std::endl;
  // RtImage im(1920, 680);
  // std::cout << im.info() << std::endl;
  // //RtTools::generateImage(sc, cam, li, im);
  // std::cout << im.getImage()[0][0] << std::endl;
  // std::cout << im.getImage()[im.getWidth() - 1][im.getHeight() - 1]
  //           << std::endl;

  // // Testing colorOfPoint from tools
  // std::cout << "########## Testing colorOfPoint from tools ##########"
  //           << std::endl;
  // RtColor c(100, 100, 100);
  // RtSphere esfera(RtVector(3., 0., 0.), 1., c);
  // RtVector ponto(2., 0., 0.);
  // RtVector doOlhoAoPonto(1., 0., 0.); // olho em (-3.,0., 0.)
  // RtVector ptLuz(-2., 0., 0.);
  // RtLight luz(ptLuz, c);
  // std::cout << "Color of point should be [67, 67, 67] and it is: "
  //           << RtTools::colorOfPoint(ponto, esfera, doOlhoAoPonto, luz)
  //           << std::endl;
  // Testing
  std::cout << "Testing image Creation" << std::endl;

  RtCamera cameraImageTest(RtVector(0, 0, 0), RtVector(1, 0, 0),
                           RtVector(0, 1, 0), 4, 4);
  std::cout << "Camera:" << cameraImageTest << std::endl;

  RtSphere sphereImageTest(RtVector(4, 0, 0), 2.0, RtColor(51, 255, 51));
  std::cout << "Sphere:" << sphereImageTest << std::endl;

  RtSphere sphereImageTest2(RtVector(4, 2, 0), 2.0, RtColor(255, 255, 0));
  std::cout << "Sphere:" << sphereImageTest2 << std::endl;

  RtScene sceneImageTest;
  sceneImageTest.add(sphereImageTest);
  sceneImageTest.add(sphereImageTest2);
  std::cout << "Scene:" << sceneImageTest << std::endl;

  RtLight lightImageTest(RtVector(0, 0, 0), RtColor(255, 255, 255));
  std::cout << "Light:" << lightImageTest << std::endl;

  RtImage imageImageTest(800, 800);
  std::cout << "Image:" << imageImageTest.info() << std::endl;

  RtTools::generateImage(sceneImageTest, cameraImageTest, lightImageTest,
                         imageImageTest);

  // std::ofstream myfile;
  // myfile.open("example.txt");
  // myfile << imageImageTest;
  // myfile.close();

  cv::Mat output;
  RtTools::convertToOpenCV(imageImageTest, output);
  RtTools::printCVImage(output);
}
