// Methods / tools for the project

#include <boost/mpi.hpp>
#include <cmath>
#include <limits>
#include <math.h>
#include <stdexcept>

#include "rt_tools.h"

namespace RtTools {
const double SHADOW_PERCENTAGE = 0.9;

bool intersection(const RtSphere &sph, const RtRay &r, RtVector &i) {
  const double DELTA = 1.0e-6;
  if ((r.getOrigin() + DELTA * r.getDirection()).distanceTo(sph.getCenter()) >
      r.getOrigin().distanceTo(sph.getCenter()))
    return false;
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
                  RtSphere &intersection_sphere,
                  RtVector &intersection_vector) {
  double mind_dist = std::numeric_limits<double>::max();
  bool intersection_found = false;
  for (unsigned int i = 0; i < scene.size(); ++i) {
    RtSphere current_sphere = scene.at_index(i);
    RtVector current_vector;
    if (intersection(current_sphere, ray, current_vector)) {
      double dist = current_sphere.getCenter().distanceTo(ray.getOrigin()) -
                    current_sphere.getRadius();
      if (dist < mind_dist) {
        mind_dist = dist;
        intersection_found = true;
        intersection_sphere = current_sphere;
        intersection_vector = current_vector;
      }
    }
  }
  return intersection_found;
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
  //RtColor colorOfLight = light.getColor();
  RtVector R = (2 * (L * N) * N - L).unit(); // reflection direction

  // Constants
  ks = 0.5; // specular
  kd = 1.0; // difuse
  ka = 0.5; // ambient
  alpha = 2.;

  // Color of my ambient
  // ra = ga = ba = 0.0;
  ra = (double)light.getColor().getR();
  ga = (double)light.getColor().getG();
  ba = (double)light.getColor().getB();

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

  if (rp > 255)
    rp = 255;
  if (gp > 255)
    gp = 255;
  if (bp > 255)
    bp = 255;
  if (rp < 0)
    rp = 0;
  if (gp < 0)
    gp = 0;
  if (bp < 0)
    bp = 0;
  return RtColor((int)rp, (int)gp, (int)bp); // the point's color
}

RtColor colorOfPoint(const RtScene &scene, const RtLight &light,
                     const RtRay &ray, const RtVector &viewer_eye,
                     const Shadows shadows) {
  RtVector intersection_point;
  RtSphere intersection_sphere;
  if (intersection(scene, ray, intersection_sphere, intersection_point)) {
    switch (shadows) {
    case Shadows::OFF: {
      return colorOfPoint(intersection_point, intersection_sphere,
                          intersection_point - viewer_eye, light);
    }
    case Shadows::ON: {
      RtRay ligth_ray(light.getPoint(), intersection_point - light.getPoint());
      RtSphere light_intersection_sphere;
      RtVector light_intersection_vector;
      // Check if light hits another sphere first
      intersection(scene, ligth_ray, light_intersection_sphere,
                   light_intersection_vector);
      if (light_intersection_vector == intersection_point) {
        return colorOfPoint(intersection_point, intersection_sphere,
                            intersection_point - viewer_eye, light);
      } else {
        return colorOfPoint(intersection_point, intersection_sphere,
                            intersection_point - viewer_eye, light)
            .darker(SHADOW_PERCENTAGE);
      }
    }
    }
  } else {
    return RtColor(255, 255, 255);
  }
}

RtColor colorOfPointRecursive(const RtScene &scene, const RtLight &light,
                              const RtRay &ray, const RtVector &viewer_eye,
                              const Shadows shadows) {
  const double DELTA = 1.0e-6;
  RtVector intersection_point;
  RtSphere intersection_sphere;
  if (intersection(scene, ray, intersection_sphere, intersection_point)) {
    RtVector normal =
        (intersection_point - intersection_sphere.getCenter()).unit();
    RtVector new_direction =
        ray.getDirection() - 2 * (ray.getDirection() * normal) * normal;
    RtVector new_viewer = -new_direction;
    RtRay new_ray(intersection_point + DELTA * new_direction, new_direction);
    switch (shadows) {
    case Shadows::OFF: {
      return (colorOfPoint(intersection_point, intersection_sphere,
                           intersection_point - viewer_eye, light) *
              (1.0 - intersection_sphere.getReflectionCoeficient())) +
             (colorOfPointRecursive(scene, light, new_ray, new_viewer,
                                    shadows) *
              intersection_sphere.getReflectionCoeficient());
    }
    case Shadows::ON: {
      RtRay ligth_ray(light.getPoint(), intersection_point - light.getPoint());
      RtSphere light_intersection_sphere;
      RtVector light_intersection_vector;
      // Check if light hits another sphere first
      intersection(scene, ligth_ray, light_intersection_sphere,
                   light_intersection_vector);
      if (light_intersection_vector == intersection_point) {
        return (colorOfPoint(intersection_point, intersection_sphere,
                             intersection_point - viewer_eye, light) *
                (1.0 - intersection_sphere.getReflectionCoeficient())) +
               (colorOfPointRecursive(scene, light, new_ray, new_viewer,
                                      shadows) *
                intersection_sphere.getReflectionCoeficient());
      } else {
        return ((colorOfPoint(intersection_point, intersection_sphere,
                              intersection_point - viewer_eye, light) *
                 (1.0 - intersection_sphere.getReflectionCoeficient())) +
                (colorOfPointRecursive(scene, light, new_ray, new_viewer,
                                       shadows) *
                 intersection_sphere.getReflectionCoeficient()))
            .darker(SHADOW_PERCENTAGE);
      }
    }
    }

  } else {
    return RtColor(255, 255, 255);
  }
}

void generateImage(const RtScene &scene, const RtCamera &camera,
                   const RtLight &light, RtImage &rtimage,
                   const Shadows shadows, const Reflection reflection) {

  RtVector intersection_point;
  RtSphere intersection_sphere;
  std::vector<std::vector<RtColor>> &image = rtimage.getImage();

  RtVector vertical_increment =
      -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

  RtVector horizontal_increment =
      -((camera.getUp().cross(camera.getTarget() - camera.getEye())).unit() *
        (camera.getWidth() / rtimage.getWidth()));

  RtVector initial_point =
      camera.getTarget() -
      (vertical_increment * 0.5 * (rtimage.getHeight() - 0.5)) -
      (horizontal_increment * 0.5 * (rtimage.getWidth() - 0.5));

  RtVector ray_direction = camera.getTarget() - camera.getEye();

  for (unsigned int i = 0; i < rtimage.getWidth(); ++i) {
    for (unsigned int j = 0; j < rtimage.getHeight(); ++j) {
      // Get point in space
      RtVector current_vector =
          initial_point + (i * horizontal_increment) + (j * vertical_increment);
      RtRay image_ray(current_vector, ray_direction);
      // RtVector current_viewer =  camera.getEye() - current_vector;

      // Find color
      switch (reflection) {
      case Reflection::OFF: {
        image[i][j] =
            colorOfPoint(scene, light, image_ray, camera.getEye(), shadows);
        break;
      }
      case Reflection::ON: {
        image[i][j] = colorOfPointRecursive(scene, light, image_ray,
                                            camera.getEye(), shadows);
        break;
      }
      }
    }
  }
}

void MPIgenerateImage(const RtScene &scene, const RtCamera &camera,
                      const RtLight &light, RtImage &rtimage,
                      const Shadows shadows, const Reflection reflection) {

  namespace mpi = boost::mpi;
  mpi::environment env;
  mpi::communicator world;
  int p = world.size();
  int rank = world.rank();

  RtVector intersection_point;
  RtSphere intersection_sphere;
  std::vector<std::vector<RtColor>> &image = rtimage.getImage();

  RtVector vertical_increment =
      -camera.getUp() * (camera.getHeight() / rtimage.getHeight());

  RtVector horizontal_increment =
      -((camera.getUp().cross(camera.getTarget() - camera.getEye())).unit() *
        (camera.getWidth() / rtimage.getWidth()));

  RtVector initial_point =
      camera.getTarget() -
      (vertical_increment * 0.5 * (rtimage.getHeight() - 0.5)) -
      (horizontal_increment * 0.5 * (rtimage.getWidth() - 0.5));

  RtVector ray_direction = camera.getTarget() - camera.getEye();

  int h = rtimage.getHeight();
  int w = rtimage.getWidth();
  int total_pixels = h * w;
  int npixels_here = total_pixels / p;
  int start;
  if (rank < total_pixels % p) {
    npixels_here++;
    start = rank * npixels_here;
  } else {
    start = rank * npixels_here + (total_pixels % p);
  }
  int pos_col = start % w;             // column position
  int pos_lin = (start - pos_col) / h; // line position
  std::vector<RtColor> colors;
  RtColor color;

  for (int j = 0; j < npixels_here; j++) {
    // Get point in space
    RtVector current_vector = initial_point + (pos_lin * horizontal_increment) +
                              (pos_col * vertical_increment);
    RtRay image_ray(current_vector, ray_direction);
    // RtVector current_viewer =  camera.getEye() - current_vector;

    // Find color
    switch (reflection) {
    case Reflection::OFF: {
      color = colorOfPoint(scene, light, image_ray, camera.getEye(), shadows);
      break;
    }
    case Reflection::ON: {
      color = colorOfPointRecursive(scene, light, image_ray, camera.getEye(),
                                    shadows);
      break;
    }
    }
    colors.push_back(color);

    if (rank == 0) {
      image[pos_lin][pos_col] = color;
    }

    start++;
    pos_col = start % w;
    pos_lin = (start - pos_col) / h;
  }

  std::vector<RtColor> other_colors;
  if (rank != 0) {
    world.send(0, 1, colors);
  } else { // rank == 0
    for (int other_rank = 1; other_rank < p; other_rank++) {
      world.recv(other_rank, 1, other_colors);

      int other_npixels_here = total_pixels / p;
      int other_start;
      if (other_rank < total_pixels % p) {
        other_npixels_here++;
        other_start = other_rank * other_npixels_here;
      } else {
        other_start = other_rank * other_npixels_here + (total_pixels % p);
      }
      int other_pos_col = other_start % w;                   // column position
      int other_pos_lin = (other_start - other_pos_col) / h; // line position

      for (int j = 0; j < other_colors.size(); j++) {
        image[other_pos_lin][other_pos_col] = other_colors[j];

        other_start++;
        other_pos_col = other_start % w;
        other_pos_lin = (other_start - other_pos_col) / h;
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
          cv::Vec3b(image[i][j].getB(), image[i][j].getG(), image[i][j].getR());

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
