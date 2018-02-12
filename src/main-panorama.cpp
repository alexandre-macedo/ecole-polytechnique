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
  double threshold = 5;
  int n_interaction = 10000;

  // panorama 1
  int n_images = 17;
  int s_index = 0;
  std::string prefix = "../img/carmel-0";
  std::string suffix = ".png";
  std::vector<cv::Mat> images1(n_images);
  for (int i = 0; i < n_images; i++) {
    std::string path = prefix + std::to_string(s_index + i) + suffix;
    images1[i] = cv::imread(path);
  }
  buildPanorama(images1, result, detector_type, threshold, n_interaction, s_index, 1);
  
  return 0;
}