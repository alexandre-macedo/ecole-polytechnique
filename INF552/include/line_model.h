#ifndef LINE_MODEL_H
#define LINE_MODEL_H

#include <cmath>
#include <memory>

#include "model.h"

typedef std::array<double, 2> Vector2VP;

class Point2D : public RANSAC::Parameter {
public:
    Vector2VP point2D;

    Point2D(double x, double y) {
        point2D[0] = x;
        point2D[1] = y;
    };

};

class LineModel : public RANSAC::Model<2> {
protected:
    double a, b, c;
    double dist_deno;

    double  m;
    double  d;

    virtual double computeDistance(std::shared_ptr<RANSAC::Parameter> param) override {
        auto extra_point = std::dynamic_pointer_cast<Point2D>(param);

        if (extra_point == nullptr) {
            throw std::runtime_error("LineModel - Passed parameters are not of type type Point2D.");
        }

        double numer = fabs(a * extra_point->point2D[0] + b * extra_point->point2D[1] + c);
        double dist = numer / dist_deno;

        return dist;
    };
public:
    LineModel(std::vector<std::shared_ptr<RANSAC::Parameter> > input_param) {
        initialize(input_param);
    }

    virtual void initialize(std::vector<std::shared_ptr<RANSAC::Parameter> > input_param) override {
        if (input_param.size() != 2) {
            throw std::runtime_error("LineModel - Number of input parameters does not match minimum number required for this model.");
        }
        auto point1 = std::dynamic_pointer_cast<Point2D>(input_param[0]);
        auto point2 = std::dynamic_pointer_cast<Point2D>(input_param[1]);

        if (point1 == nullptr || point2 == nullptr) {
            throw std::runtime_error("LineModel - input_param type mismatch. It is not a Point2D.");
        }

        std::copy(input_param.begin(), input_param.end(), min_model_param.begin());

        m = (point2->point2D[1] - point1->point2D[1]) / (point2->point2D[0] - point1->point2D[0]);
        d = point1->point2D[1] - m * point1->point2D[0];

        a = m;
        b = -1.0;
        c = d;

        dist_deno = sqrt(a*a + b*b);
    }

    virtual std::pair<double, std::vector<std::shared_ptr<RANSAC::Parameter> > > evaluate(
            std::vector<std::shared_ptr<RANSAC::Parameter> > evaluate_param, double threshold) override{

        std::vector<std::shared_ptr<RANSAC::Parameter> > inliers;
        int n_total_param = int(evaluate_param.size());
        int n_inliers = 0;

        for(auto& param : evaluate_param) {
            if (computeDistance(param) < threshold) {
                inliers.push_back(param);
                n_inliers++;
            }
        }

        double inliers_fraction = double(n_inliers) / double(n_total_param);
        return std::make_pair(inliers_fraction, inliers);
    };

};

#endif //LINE_MODEL_H
