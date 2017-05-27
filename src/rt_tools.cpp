// Methods / tools for the project

#include <cmath>
#include <limits>
#include <math.h>
#include <stdexcept>

#include "rt_tools.h"

namespace RtTools {
bool intersection(const RtSphere &sph, const RtRay &r, RtVector &i) {
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

bool intersection(const RtScene &scene, const RtRay &ray,
                  RtSphere &intersectionSphere, RtVector &intersectionVector) {
  double minDist = std::numeric_limits<double>::max();
  bool intersectionFound = false;
  for (unsigned int i = 0; i < scene.size(); ++i) {
    RtSphere currentSphere = scene.at_index(i);
    RtVector currentVector;
    if (intersection(currentSphere, ray, currentVector)) {
      double dist = currentSphere.getCenter().distanceTo(ray.getOrigin()) -
                    currentSphere.getRadius();
      if (dist < minDist) {
        minDist = dist;
        intersectionFound = true;
        intersectionSphere = currentSphere;
        intersectionVector = currentVector;
      }
    }
  }
  return intersectionFound;
}

// "viewer" is the vector from the camera pointing towards the point
// using Phong Reflection Model -
// https://en.wikipedia.org/wiki/Phong_reflection_model
RtColor colorOfPoint(const RtVector &pt, const RtSphere &sph,
                     const RtVector &viewer, const RtLight &light) {
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
  ks = 1.0;
  kd = 1.0;
  ka = 1.0;
  alpha = 2.;

  // Color of my ambient
  ra = ga = ba = 0.0;

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

// void generateSimpleImage(const RtScene &scene, const RtCamera &camera,
//                          const RtLight &light, RtImage &rtimage) {
//   RtVector intersectionPoint;
//   RtSphere intersectionSphere;
//   std::vector<std::vector<RtColor>> &image = rtimage.getImage();

//   RtVector verticalIncrement =
//       -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

//   RtVector horizontalIncrement =
//       -((camera.getUp().cross(camera.getTarget() - camera.getEye())).unit() *
//         (camera.getWidth() / rtimage.getWidth()));

//   RtVector initialPoint =
//       camera.getTarget() -
//       (verticalIncrement * 0.5 * (rtimage.getHeight() - 0.5)) -
//       (horizontalIncrement * 0.5 * (rtimage.getWidth() - 0.5));

//   RtVector rayDirection = camera.getTarget() - camera.getEye();

//   for (unsigned int i = 0; i < rtimage.getWidth(); ++i) {
//     for (unsigned int j = 0; j < rtimage.getHeight(); ++j) {
//       // Get point in space
//       RtVector currentVector =
//           initialPoint + (i * horizontalIncrement) + (j * verticalIncrement);
//       RtRay currentRay(currentVector, rayDirection);
//       RtVector currentViewer = currentVector - camera.getEye();

//       // Find color
//       if (intersection(scene, currentRay, intersectionSphere,
//                        intersectionPoint)) {
//         image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
//                                    currentViewer, light);
//       } else {
//         image[i][j] = RtColor(255, 255, 255);
//       }
//     }
//   }
// }

// void generateImageWithShadows(const RtScene &scene, const RtCamera &camera,
//                               const RtLight &light, RtImage &rtimage) {
//   RtVector intersectionPoint;
//   RtSphere intersectionSphere;
//   std::vector<std::vector<RtColor>> &image = rtimage.getImage();

//   RtVector verticalIncrement =
//       -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

//   RtVector horizontalIncrement =
//       -((camera.getUp().cross(camera.getTarget() - camera.getEye())).unit() *
//         (camera.getWidth() / rtimage.getWidth()));

//   RtVector initialPoint =
//       camera.getTarget() -
//       (verticalIncrement * 0.5 * (rtimage.getHeight() - 0.5)) -
//       (horizontalIncrement * 0.5 * (rtimage.getWidth() - 0.5));

//   RtVector rayDirection = camera.getTarget() - camera.getEye();

//   for (unsigned int i = 0; i < rtimage.getWidth(); ++i) {
//     for (unsigned int j = 0; j < rtimage.getHeight(); ++j) {
//       // Get point in space
//       RtVector currentVector =
//           initialPoint + (i * horizontalIncrement) + (j * verticalIncrement);
//       RtRay imageRay(currentVector, rayDirection);
//       RtVector currentViewer = currentVector - camera.getEye();

//       // Find color
//       if (intersection(scene, imageRay, intersectionSphere,
//                        intersectionPoint)) {
//         RtRay lightRay(light.getPoint(), intersectionPoint -
//         light.getPoint());
//         RtSphere lightIntersectionSphere;
//         RtVector lightIntersectionVector;
//         // Check if light hits another sphere first
//         if (intersection(scene, lightRay, lightIntersectionSphere,
//                          lightIntersectionVector)) {
//           if (lightIntersectionVector == intersectionPoint) {
//             image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
//                                        currentViewer, light);
//           } else {
//             image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
//                                        currentViewer, light)
//                               .darker(0.5);
//           }
//         }

//       } else {
//         image[i][j] = RtColor(255, 255, 255);
//       }
//     }
//   }
// }

void generateImage(const RtScene &scene, const RtCamera &camera,
                   const RtLight &light, RtImage &rtimage, imageType imType) {
  RtVector intersectionPoint;
  RtSphere intersectionSphere;
  std::vector<std::vector<RtColor>> &image = rtimage.getImage();

  RtVector verticalIncrement =
      -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

  RtVector horizontalIncrement =
      -((camera.getUp().cross(camera.getTarget() - camera.getEye())).unit() *
        (camera.getWidth() / rtimage.getWidth()));

  RtVector initialPoint =
      camera.getTarget() -
      (verticalIncrement * 0.5 * (rtimage.getHeight() - 0.5)) -
      (horizontalIncrement * 0.5 * (rtimage.getWidth() - 0.5));

  RtVector rayDirection = camera.getTarget() - camera.getEye();

  for (unsigned int i = 0; i < rtimage.getWidth(); ++i) {
    for (unsigned int j = 0; j < rtimage.getHeight(); ++j) {
      // Get point in space
      RtVector currentVector =
          initialPoint + (i * horizontalIncrement) + (j * verticalIncrement);
      RtRay imageRay(currentVector, rayDirection);
      RtVector currentViewer = currentVector - camera.getEye();

      // Find color
      switch (imType) {
      case imageType::SIMPLE: {
        if (intersection(scene, imageRay, intersectionSphere,
                         intersectionPoint)) {
          image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
                                     currentViewer, light);
        } else {
          image[i][j] = RtColor(255, 255, 255);
        }
        break;
      }
      case imageType::WITHSHADOWS: {
        if (intersection(scene, imageRay, intersectionSphere,
                         intersectionPoint)) {
          RtRay lightRay(light.getPoint(),
                         intersectionPoint - light.getPoint());
          RtSphere lightIntersectionSphere;
          RtVector lightIntersectionVector;
          // Check if light hits another sphere first
          if (intersection(scene, lightRay, lightIntersectionSphere,
                           lightIntersectionVector)) {
            if (lightIntersectionVector == intersectionPoint) {
              image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
                                         currentViewer, light);
            } else {
              image[i][j] = colorOfPoint(intersectionPoint, intersectionSphere,
                                         currentViewer, light)
                                .darker(0.5);
            }
          }

        } else {
          image[i][j] = RtColor(255, 255, 255);
        }
        break;
      }
      case imageType::WITHREFLECTION: {
        std::cout << "TODO" << std::endl;
        break;
      }
      }
    }
  }
}

void convertToOpenCV(const RtImage &input, cv::Mat &output) {

  std::vector<std::vector<RtColor>> image = input.getConstImage();
  cv::Mat cvimage(input.getHeight(), input.getWidth(), CV_8UC3,
                  cv::Scalar(255, 255, 255));

  for (unsigned int i = 0; i < input.getWidth(); ++i)
    for (unsigned int j = 0; j < input.getHeight(); ++j)
      cvimage.at<cv::Vec3b>(cv::Point(i, j)) =
          cv::Vec3b(image[i][j].getR(), image[i][j].getG(), image[i][j].getB());

  output = cvimage.clone();
}

void printCVImage(const cv::Mat &output) {
  if (!output.data) // Check for invalid input
  {
    std::cout << "Could not open or find the output" << std::endl;
  } else {
    cv::namedWindow("Ray Tracer",
                    cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::imshow("Ray Tracer", output);     // Show our image inside it.

    cv::waitKey(0); // Wait for a keystroke in the window
  }
}

void saveCVImage(const cv::Mat &output, const std::string &name) {
  if (!output.data) // Check for invalid input
  {
    std::cout << "Could not open or find the output." << std::endl;
  } else {
    cv::imwrite(name + ".jpg", output);
  }
}
}
