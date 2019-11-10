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

  // // Testing Image Creation

  std::cout << "Testing image Creation" << std::endl;

  RtCamera camera_image_test(RtVector(0, 3, 4), RtVector(1, 3, 4),
                             RtVector(0, 1, 0), 30, 20);
  std::cout << "Camera:" << camera_image_test << std::endl;

  RtSphere sphere_image_test(RtVector(4, 0, 0), 2.0, RtColor(51, 255, 51), 0.0);
  std::cout << "Sphere:" << sphere_image_test << std::endl;

  RtSphere sphere_image_test2(RtVector(6, 3, 0), 2.0, RtColor(51, 255, 255),
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
  
  RtTools::generateImage(scene_image_test, camera_image_test,
                            light_image_test, image_image_test,
                            RtTools::Shadows::ON, RtTools::Reflection::ON);


  cv::Mat output;
  RtTools::convertToOpenCV(image_image_test, output);
  RtTools::printCVImage(output);
}