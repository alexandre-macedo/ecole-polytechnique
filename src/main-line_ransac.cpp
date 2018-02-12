#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <random>

#include "ransac.h"
#include "line_model.h"

double computeSlope(int x0, int y0, int x1, int y1) {
    return (double) (y1-y0) / (x1-x0);
}

void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int line_width) {
    double slope = computeSlope(a.x, a.y, b.x, b.y);
    cv::Point p(0,0), q(img.cols, img.rows);

    p.y = -(a.x - p.x) * slope + a.y;
    q.y = -(b.x - q.x) * slope + b.y;

    cv::line(img, p, q, color, line_width, 8, 0);
}

int main(int argc, char* argv[]) {

    int side = 500;
    int n_points = 500;

    cv::Mat canvas(side, side, CV_8UC3);
    canvas.setTo(255);

    std::random_device seed;
    std::mt19937 random_number_generator = std::mt19937(seed());

    std::uniform_int_distribution<int> uni_dist(0, side-1);

    int perturb = 50;
    std::normal_distribution<double> pertub_dist(0, perturb);

    std::vector<std::shared_ptr<RANSAC::Parameter> > cand_points;

    for (int i = 0; i < n_points; i++) {
        int diag = uni_dist(random_number_generator);
        cv::Point pt((int) floor(side - diag + pertub_dist(random_number_generator)), (int) floor(diag + pertub_dist(random_number_generator)));
        cv::circle(canvas, pt, int(floor(side / 200)), cv::Scalar(0, 0, 255), -1);

        std::shared_ptr<RANSAC::Parameter> cand_pt = std::make_shared<Point2D>(pt.x, pt.y);
        cand_points.push_back(cand_pt);
    }

    RANSAC::Estimator<LineModel, 2> estimator;
    estimator.initialize(20,100);
    //int start = cv::getTickCount();
    estimator.estimate(cand_points);
    //int end = cv::getTickCount();
    //std::cout << "RANSAC ran in " << double(end-start) / double(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;

    auto best_inliner = estimator.getBestInliers();
    if (best_inliner.size() > 0) {
        for (auto& inlier : best_inliner) {
            auto r_pt = std::dynamic_pointer_cast<Point2D>(inlier);
            cv::Point pt((int)floor(r_pt->point2D[0]), (int) floor(r_pt->point2D[1]));
            cv::circle(canvas, pt, int(floor(side/200)), cv::Scalar(0, 255, 0), -1);
        }

        auto best_line = estimator.getBestModel();
        if(best_line) {
            auto best_line_pt1 = std::dynamic_pointer_cast<Point2D>(best_line->getModelParams()[0]);
            auto best_line_pt2 = std::dynamic_pointer_cast<Point2D>(best_line->getModelParams()[1]);
            if (best_line_pt1 && best_line_pt2) {
                cv::Point pt1((int)best_line_pt1->point2D[0], (int)best_line_pt1->point2D[1]);
                cv::Point pt2((int)best_line_pt2->point2D[0], (int)best_line_pt2->point2D[1]);
                DrawFullLine(canvas, pt1, pt2, cv::Scalar(0,0,0), 2);
            }
        }
        
        // Displaying
        cv::imshow("RANSAC example", canvas);
        cv::waitKey(0);
        //cv::imwrite("RANSAC_line_fit.png", canvas);
        return 0;
    }

}