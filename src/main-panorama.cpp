#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <random>
#include <cmath>

#include "homography_model.h"

int main(int argc, char* argv[]) {
  cv::Mat result;
  KeypointDetectorType detector_type = AKAZE;
  double threshold = 1;
  int n_interaction = 50000;

  int n_images = 3;
  int s_index = 0;
  std::string prefix = "../img/carmel-0";
  std::string suffix = ".png";
  std::vector<cv::Mat> images1(n_images);
  for (int i = 0; i < n_images; i++) {
    std::string path = prefix + std::to_string(s_index + i) + suffix;
    images1[i] = cv::imread(path);
  }
  buildPanorama(images1, result, detector_type, threshold, n_interaction, s_index, 1);

  cv::Mat result2;
  n_images = 4;
  s_index = 14;
  std::vector<cv::Mat> images2(n_images);
  for (int i = 0; i < n_images; i++) {
    std::string path = prefix + std::to_string(s_index + i) + suffix;
    images2[i] = cv::imread(path);
  }
  buildPanorama(images2, result2, detector_type, threshold, n_interaction, s_index, 1);
  
  return 0;
}