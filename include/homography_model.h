#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <random>
#include <cmath>

#include "ransac2.h"

class Point2DPair : public RANSAC::Parameter {
public:
  const double x1, y1, x2, y2;
  Point2DPair(double x1, double y1, double x2, double y2) : x1(x1), y1(y1), x2(x2), y2(y2) {};
};

class HomographyModel : public RANSAC::Model<4> {
public:
  static const int MODEL_SIZE = 4;

  HomographyModel(const std::vector<RANSAC::Parameter*>& input_parameters) {
    initialize(input_parameters);
  }

  virtual void initialize(const std::vector<RANSAC::Parameter*>& input_parameters) override {
    if (input_parameters.size() != MODEL_SIZE) {
      throw std::runtime_error("HomographyModel - wrong size for input parameters");
    }

    std::copy(input_parameters.begin(), input_parameters.end(), min_model_param.begin());

    cv::Mat A(A_ROWS, A_COLS, CV_64F);
    cv::Mat b(A_ROWS, 1, CV_64F);
    computeAb(A, b, input_parameters);
    computeH(A, b);
  }

  virtual std::pair<double, std::vector<RANSAC::Parameter*> > evaluate(
    std::vector<RANSAC::Parameter*> evaluate_param, double threshold) override {
    std::vector<RANSAC::Parameter*> inliers;
    int n_total_param = int(evaluate_param.size());
    int n_inliers = 0;
    for (RANSAC::Parameter*& parameter : evaluate_param) {
      if (computeDistance(parameter) < threshold) {
        inliers.push_back(parameter);
        n_inliers++;
      }
    }

    double inliers_fraction = double(n_inliers) / double(n_total_param);
    return std::make_pair(inliers_fraction, inliers);
  }

  cv::Mat* getHomography() {
    return H;
  }

protected:
  virtual double computeDistance(RANSAC::Parameter* parameter) override {
    Point2DPair* match = dynamic_cast<Point2DPair*>(parameter);

    double w = 1. / (H->at<double>(2, 0) * match->x1 + H->at<double>(2, 1) * match->y1 + 1.);
    double dx = (H->at<double>(0, 0) * match->x1 + H->at<double>(0, 1) * match->y1 + H->at<double>(0, 2))*w - match->x2;
    double dy = (H->at<double>(1, 0) * match->x1 + H->at<double>(1, 1) * match->y1 + H->at<double>(1, 2))*w - match->y2;

    return sqrt(dx*dx + dy*dy);
  };

private:
  static const int H_ROWS = 3;
  static const int H_COLS = 3;
  static const int H_DIM = 9;
  static const int A_ROWS = 2 * MODEL_SIZE;
  static const int A_COLS = H_DIM - 1;

  cv::Mat* H;

  void computeAb(cv::Mat& A, cv::Mat& b, const std::vector<RANSAC::Parameter*>& input_parameters) {
    for (int i = 0; i < MODEL_SIZE; i++) {
      Point2DPair* match = dynamic_cast<Point2DPair*>(input_parameters[i]);
      if (match == NULL) {
        throw std::runtime_error("HomographyModel - parameter mismatch (expected Point2DPair)");
      }

      // line 2*i
      int index = 2 * i;
      A.at<double>(index, 0) = match->x1;
      A.at<double>(index, 1) = match->y1;
      A.at<double>(index, 2) = 1.;
      A.at<double>(index, 3) = 0.;
      A.at<double>(index, 4) = 0.;
      A.at<double>(index, 5) = 0.;
      A.at<double>(index, 6) = -match->x1 * match->x2;
      A.at<double>(index, 7) = -match->x2 * match->y1;

      b.at<double>(index, 0) = match->x2;

      // line 2*i + 1
      index++;
      A.at<double>(index, 0) = 0.;
      A.at<double>(index, 1) = 0.;
      A.at<double>(index, 2) = 0.;
      A.at<double>(index, 3) = match->x1;
      A.at<double>(index, 4) = match->y1;
      A.at<double>(index, 5) = 1.;
      A.at<double>(index, 6) = -match->x1 * match->y2;
      A.at<double>(index, 7) = -match->y1 * match->y2;

      b.at<double>(index, 0) = match->y2;
    }
  }

  void computeH(const cv::Mat& A, const cv::Mat& b) {
    H = new cv::Mat(H_ROWS, H_COLS, CV_64F);
    cv::Mat auxH(H_DIM - 1, 1, CV_64F);
    cv::solve(A, b, auxH);
    for (int i = 0; i < H_ROWS; i++)
      for (int j = 0; j < H_COLS; j++)
        if (i != 2 || j != 2)
          H->at<double>(i, j) = auxH.at<double>(i * H_COLS + j, 0);
    H->at<double>(H_ROWS - 1, H_COLS - 1) = 1;
  }
};

enum KeypointDetectorType {AKAZE, ORB};

void computeKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& kpts, cv::Mat& descriptor, KeypointDetectorType detector_type) {
  if (detector_type == AKAZE) {
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    akaze->detectAndCompute(image, cv::noArray(), kpts, descriptor);
  } else if (detector_type == ORB) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detectAndCompute(image, cv::noArray(), kpts, descriptor);
  }
}

void matchPoints(const cv::Mat& descriptor1, const cv::Mat& descriptor2, std::vector<cv::DMatch>& matches){
  cv::BFMatcher M(cv::NORM_L2);
  M.match(descriptor1, descriptor2, matches);
}

void drawMatches(
    const cv::Mat& image1, const std::vector<cv::KeyPoint>& key_points1,
    const cv::Mat& image2, const std::vector<cv::KeyPoint>& key_points2,
    std::vector<cv::DMatch>& matches) {
  cv::Mat J;
  drawMatches(image1, key_points1, image2, key_points2, matches, J);
  resize(J, J, cv::Size(), .5, .5);
  imshow("Matches", J);
  cv::waitKey(0);
}

void findHomography(
  const cv::Mat& image1, const cv::Mat& image2,
  KeypointDetectorType detector_type, double threshold, int num_inter, cv::Mat& H, 
  double horizontal_fraction) {
  std::vector<cv::KeyPoint> key_points1, key_points2;
  cv::Mat descriptor1, descriptor2;
  std::vector<cv::DMatch> matches;

  int dx = image1.cols - image2.cols * horizontal_fraction;
  cv::Rect roi(dx, 0, image2.cols * horizontal_fraction, image1.rows);
  computeKeyPoints(image1(roi), key_points1, descriptor1, detector_type);
  // computeKeyPoints(image1, key_points1, descriptor1, detector_type);
  computeKeyPoints(image2, key_points2, descriptor2, detector_type);
  std::cout << "> computed key points" << std::endl;

  matchPoints(descriptor1, descriptor2, matches);
  std::cout << "> computed matches" << std::endl;
  // drawMatches(image1, key_points1, image2, key_points2, matches);
  
  std::vector<RANSAC::Parameter*> candidate_matches;
  for (size_t i = 0; i < matches.size(); i++) {
    Point2DPair* match = new Point2DPair(
      key_points1[matches[i].queryIdx].pt.x + dx, key_points1[matches[i].queryIdx].pt.y,
      key_points2[matches[i].trainIdx].pt.x, key_points2[matches[i].trainIdx].pt.y);
    candidate_matches.push_back(match);
  }

  RANSAC::Estimator<HomographyModel, HomographyModel::MODEL_SIZE> estimator;
  std::vector<RANSAC::Parameter*> inliers;
  HomographyModel* best_homo;
  estimator.initialize(threshold, num_inter);
  estimator.estimate(candidate_matches);

  inliers = estimator.getBestInliers();
  best_homo = dynamic_cast<HomographyModel*>(estimator.getBestModel());
  if (!best_homo) {
    std::cout << "Could not compute homography" << std::endl;
  } else {
    H = (*(best_homo->getHomography()));

    std::cout << "Cmputed homography" << std::endl;
    std::cout << "RANSAC took: " << estimator.getExecutionTime() << " ms" << std::endl;
    // std::cout << "\t->HOMOGRAPHY = " << std::endl << H << std::endl;
    std::cout << inliers.size() << " inliers out of " << matches.size() << " matches" << std::endl;
  }
}

// Before stitching image1 to image2, the right offsets should be computed in
// order to consider the ROI (region of interest) and stitch the images correctly
cv::Mat stitch(const cv::Mat &image1, const cv::Mat &image2, const cv::Mat &H) {
  // coordinates of the 4 corners of the image
  std::vector<cv::Point2f> corners(4);
  corners[0] = cv::Point2f(0, 0);
  corners[1] = cv::Point2f(0, image2.rows);
  corners[2] = cv::Point2f(image2.cols, 0);
  corners[3] = cv::Point2f(image2.cols, image2.rows);

  std::vector<cv::Point2f> corner_transf(4);
  cv::perspectiveTransform(corners, corner_transf, H);

  double offsetX = 0.0;
  double offsetY = 0.0;

  // get max offset outside of the image
  for (size_t i = 0; i < 4; i++) {
    // std::cout << "corner_transf[" << i << "] =" << corner_transf[i] << std::endl;
    if (corner_transf[i].x < offsetX) {
      offsetX = corner_transf[i].x;
    }

    if (corner_transf[i].y < offsetY) {
      offsetY = corner_transf[i].y;
    }
  }

  offsetX = -offsetX;
  offsetY = -offsetY;
  // std::cout << "offsetX = " << offsetX << " ; offsetY = " << offsetY << std::endl;

  // get max width and height for the new size of the panorama
  double maxX = std::max((double)image1.cols + offsetX, (double)std::max(corner_transf[2].x, corner_transf[3].x) + offsetX);
  double maxY = std::max((double)image1.rows + offsetY, (double)std::max(corner_transf[1].y, corner_transf[3].y) + offsetY);
  // std::cout << "maxX = " << maxX << " ; maxY = " << maxY << std::endl;

  cv::Size size_warp(maxX, maxY);
  cv::Mat panorama(size_warp, CV_8UC3);

  // create the transformation matrix to be able to have all the pixels
  cv::Mat H2 = cv::Mat::eye(3, 3, CV_64F);
  H2.at<double>(0, 2) = offsetX;
  H2.at<double>(1, 2) = offsetY;

  cv::warpPerspective(image2, panorama, H2*H, size_warp);

  // ROI for image1
  cv::Rect image1_rect(offsetX, offsetY, image1.cols, image1.rows);
  
  // copy image1 in the panorama using the ROI
  cv::Mat half = cv::Mat(panorama, image1_rect);
  image1.copyTo(half);

  // create the new mask matrix for the panorama
  cv::Mat mask = cv::Mat::ones(image2.size(), CV_8U) * 255;
  cv::warpPerspective(mask, mask, H2*H, size_warp);
  cv::rectangle(mask, image1_rect, cv::Scalar(255), -1);

  return panorama;
}

void buildPanorama(
    std::vector<cv::Mat>& images, cv::Mat& result,
    KeypointDetectorType detector_type, double threshold, int num_inter, int s_index, 
    double horizontal_fraction) {
  if (images.size() < 2) {
    return;
  }

  result = images[0];
  for (size_t i = 1; i < images.size(); i++) {
    std::string index = "[" + std::to_string(s_index) + ".." + std::to_string(s_index + i) + "]";
    std::cout << "Creating panorama of images" << index << std::endl;
    cv::Mat H;
    findHomography(result, images[i], detector_type, threshold, num_inter, H, horizontal_fraction);
    result = stitch(images[i], result, H);
    std::cout << "Stitched panorama of images " << index << std::endl;
    cv::imwrite("panorama " + index + ".png", result);
    std::cout << "Saved to file \"panorama " << index << ".png\"" << std::endl << std::endl;
  }
}