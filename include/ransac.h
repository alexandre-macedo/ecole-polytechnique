#ifndef RANSAC_H
#define RANSAC_H

#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <algorithm>
#include <vector>
#include <memory>

#include "model.h"

namespace RANSAC {
    template <class T, int model_size>
    class Estimator {
    private:
        std::vector<std::shared_ptr<Parameter> > data;

        std::vector<std::shared_ptr<T> > sampled_models;
        std::shared_ptr<T> best_model;
        std::vector<std::shared_ptr<Parameter> > best_inliers;

        int max_iterations;
        double threshold;
        double best_score;
        int best_model_index;

    public:
        void reset(void) {
            data.clear();
            sampled_models.clear();

            best_model_index = -1;
            best_score = 0.0;
        };

        void initialize(double threshold, int max_iterations = 1000) {
            this->threshold = threshold;
            this->max_iterations = max_iterations;
        }

        std::shared_ptr<T> getBestModel(void) {
            return best_model;
        }

        const std::vector<std::shared_ptr<Parameter> >& getBestInliers(void) {
            return best_inliers;
        }

        bool estimate(std::vector<std::shared_ptr<Parameter> > data) {
            if(data.size() <= model_size) {
                std::cout << "Number of data points is too small." << std::endl;
                return false;
            }

            this->data = data;
            std::uniform_int_distribution<int> uniDist(0, int(this->data.size()-1));

            std::vector<double> inliers_frac_acc(max_iterations);
            std::vector<std::vector<std::shared_ptr<Parameter> > > inliers_acc(max_iterations);

            sampled_models.resize(max_iterations);

            for (int i = 0; i < max_iterations; ++i) {
                std::vector<std::shared_ptr<Parameter> > random_samples(model_size);
                std::vector<std::shared_ptr<Parameter> > remainder_samples = this->data;

                std::random_device seed;
                std::mt19937 rng(seed());
                std::shuffle(remainder_samples.begin(), remainder_samples.end(), rng);
                std::copy(remainder_samples.begin(), remainder_samples.begin() + model_size, random_samples.begin());
                remainder_samples.erase(remainder_samples.begin(), remainder_samples.begin() + model_size);

                std::shared_ptr<T> random_model = std::make_shared<T>(random_samples);

                std::pair<double, std::vector<std::shared_ptr<Parameter> > > eval_pair = random_model->evaluate(remainder_samples, threshold);
                inliers_frac_acc[i] = eval_pair.first;
                inliers_acc[i] = eval_pair.second;

                sampled_models[i] = random_model;
            }

            for (int i = 0; i < max_iterations; ++i) {
                if (inliers_frac_acc[i] > best_score) {
                    best_score = inliers_frac_acc[i];
                    best_model_index = sampled_models.size() - 1;
                    best_model = sampled_models[i];
                    best_inliers = inliers_acc[i];
                }
            }
            
            reset();
            return true;
        }
    };
}
#endif //RANSAC_H
