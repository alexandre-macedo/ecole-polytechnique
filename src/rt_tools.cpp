// Methods / tools for the project

#include <cmath>
#include <math.h>
#include <stdexcept>

#include "rt_tools.h"

namespace RtTools {
bool intersection(RtSphere &sph, RtRay &r, RtVector &i) {
  RtVector center = sph.getCenter();
  double radius = sph.getRadius();
  RtVector dir = r.getDirection();
  dir = dir.unit(); // norm == 1 now
  RtVector p = r.getOrigin();
  double d = ((dir.cross(center - p)).norm2());

  if (d > radius) {
    return false;
  }

  double hip = (center - p).norm2();
  double cateto = sqrt(hip * hip - d * d);
  double correction = sqrt(radius * radius - d * d);
  // Updating intersection point
  // std::cout << "p: " << p << " dir: " << dir << " cat: " << cateto << " dist:
  // " << d << std::endl;;
  i = p + dir * (cateto - correction);

  return true;
}

// "viewer" is the vector from the camera pointing towards the point
// using Phong Reflection Model -
// https://en.wikipedia.org/wiki/Phong_reflection_model
RtColor colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer,
                     const RtLight &light) {
  if (round(pt.distanceTo(sph.getCenter())) != sph.getRadius()) {
    throw std::runtime_error(
        "Point must be in the surface of the sphere in colorOfPoint.");
  }

  double ks, kd, ka, alpha;
  double ra, ga, ba; // red, green and blue of the ambient
  // int rl, gl, bl; // red, green and blue of Light
  double rs, gs, bs; // red, green and blue of Sphere
  double rp, gp, bp; // same for our point
  RtColor cSphere;   // color of the sphere

  RtVector V = ((-1) * viewer).unit();         // viewer
  RtVector N = (pt - sph.getCenter()).unit();  // normal
  RtVector L = (light.getPoint() - pt).unit(); // light
  RtColor colorOfLight = light.getColor();
  RtVector R = (2 * (L * N) * N - L).unit(); // reflection direction

  // Constants
  ks = 0.25;
  kd = 0.5;
  ka = 0.5;
  alpha = 2.;

  // Color of my ambient
  ra = ga = ba = 50.0;

  // rl = colorOfLight.getR();
  // gl = colorOfLight.getG();
  // bl = colorOfLight.getB();

  cSphere = sph.getColor();
  rs = (double)cSphere.getR();
  gs = (double)cSphere.getG();
  bs = (double)cSphere.getB();

  // Percentage for diffusion

  rp = ka * ra;
  gp = ka * ga;
  bp = ka * ba;
  // std::cout << L * N << std::endl;
  if ((L * N) >= 0.0) {
    double pForDiffusion = .7;
    // std::cout << rp << std::endl;
    rp += kd * (L * N) * (pForDiffusion * rs);
    // std::cout << rp << std::endl;
    gp += kd * (L * N) * (pForDiffusion * gs);
    bp += kd * (L * N) * (pForDiffusion * bs);
    if ((R * V) >= 0.0) {
      rp += ks * pow(R * V, alpha) * (1 - pForDiffusion) * rs;
      gp += ks * pow(R * V, alpha) * (1 - pForDiffusion) * gs;
      bp += ks * pow(R * V, alpha) * (1 - pForDiffusion) * bs;
    }
  }

  return RtColor((int)rp, (int)gp, (int)bp); // the point's color
}


void generateImage(const RtScene &scene, const RtCamera &camera,
                   const RtLight &light, RtImage &rtimage) {
  RtVector intersectionPoint;
  bool intersectionFound = false;
  RtSphere currentSphere;
  std::vector<std::vector<RtColor>> &image = rtimage.getImage();

  RtVector verticalIncrement =
      -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

  RtVector horizontalIncrement =
      (camera.getUp().cross(camera.getTarget() - camera.getEye())) *
      (camera.getWidth() / rtimage.getWidth());

  RtVector initialPoint =
      camera.getTarget() + (camera.getUp() * camera.getHeight() * 0.5 *
                            (1.0 - 1.0 / rtimage.getHeight())) -
      (camera.getUp().cross(camera.getTarget() - camera.getEye()) *
       camera.getWidth() * 0.5 * (1.0 - 1.0 / rtimage.getWidth()));

  for (unsigned int i = 0; i < rtimage.getWidth(); ++i) {
    for (unsigned int j = 0; j < rtimage.getHeight(); ++j) {
      intersectionFound = false;
      // Get point in space
      RtVector currentVector =
          initialPoint + i * horizontalIncrement + j * verticalIncrement;
      RtRay currentRay(camera.getEye(), currentVector - camera.getEye());
      RtVector viewer = currentVector - camera.getEye();
      // Find Intersection point
      for (unsigned int k = 0; k < scene.size(); ++k) {
        currentSphere = scene.at_index(k);
        if (intersection(currentSphere, currentRay, intersectionPoint)) {
          intersectionFound = true;
          break;
        }
      }
      // Find color
      if (intersectionFound) {
        //  std::cout << "Intersection point " << intersectionPoint <<
        //  std::endl;
        //  std::cout << "Sphere " << currentSphere << std::endl;
        //  std::cout << "Viewer " << viewer << std::endl;
        //  std::cout << colorOfPoint(intersectionPoint, currentSphere, viewer,
        //                            light)
        //            << std::endl;
        image[i][j] =
            colorOfPoint(intersectionPoint, currentSphere, viewer, light);
      } else {
        image[i][j] = RtColor(255, 255, 255);
      }
    }
  }
}

void convertToOpenCV(RtImage &input, cv::Mat &output) {

  std::vector<std::vector<RtColor>> &image = input.getImage();
  cv::Mat cvimage(input.getHeight(), input.getWidth(), CV_8UC3,
                  cv::Scalar(255, 255, 255));
  for (unsigned int i = 0; i < input.getWidth(); ++i)
    for (unsigned int j = 0; j < input.getHeight(); ++j)
      cvimage.at<cv::Vec3b>(cv::Point(i, j)) =
          cv::Vec3b(image[i][j].getR(), image[i][j].getG(), image[i][j].getB());

  //    cv::Scalar(image[i][j].getR(), image[i][j].getG(), image[i][j].getB());

  output = cvimage.clone();
}

void printCVImage(cv::Mat &output) {
  if (!output.data) // Check for invalid input
  {
    std::cout << "Could not open or find the output" << std::endl;
  }

  cv::namedWindow("Ray Tracer",
                  cv::WINDOW_AUTOSIZE); // Create a window for display.
  cv::imshow("Ray Tracer", output); // Show our image inside it.

  cv::waitKey(0); // Wait for a keystroke in the window
}
}
