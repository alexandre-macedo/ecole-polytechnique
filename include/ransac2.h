#ifndef RANSAC2_H
#define RANSAC2_H

#include <vector>
#include <array>

namespace RANSAC {
  class Parameter {
  public:
    virtual ~Parameter() {}
  };

  template <int model_size> class Model {
  public:
    virtual void initialize(const std::vector<Parameter*>& input_param) = 0;
    virtual std::pair<double, std::vector<Parameter*> > evaluate(
        std::vector<Parameter*> input_param, double threshold) = 0;

    virtual std::array<Parameter*, model_size> getModelParameters() {
      return min_model_param;
    };

  protected:
    std::array<Parameter*, model_size> min_model_param;

    virtual double computeDistance(Parameter* parameter) = 0;
  };
  
  template <class T, int model_size> class Estimator {
  public:
    void initialize(double threshold, int max_iterations = 1000) {
      this->threshold = threshold;
      this->max_iterations = max_iterations;
    }

    bool estimate(const std::vector<Parameter*>& data) {
      if (data.size() <= model_size) {
        std::cout << "Insuficient data to compute best model." << std::endl;
        execution_time = -1;
        return false;
      }

      int64 start_time = cv::getTickCount();
      this->data = data;
      std::uniform_int_distribution<int> uni_dist(0, int(this->data.size() - 1));
      
      for (int i = 0; i < max_iterations; i++) {
        std::vector<Parameter*> random_samples(model_size);
        std::vector<Parameter*> remainder_samples = this->data;
        std::random_device seed;
        std::mt19937 g(seed());
        std::shuffle(remainder_samples.begin(), remainder_samples.end(), g);
        std::copy(
            remainder_samples.begin(), remainder_samples.begin() + model_size, random_samples.begin());
        remainder_samples.erase(remainder_samples.begin(), remainder_samples.begin() + model_size);

        T randomModel(random_samples);
        std::pair<double, std::vector<Parameter*> > eval_pair = 
            randomModel.evaluate(remainder_samples, threshold);
        double score = eval_pair.first;

        if (score > best_score) {
          best_score = score;
          best_model = new T(random_samples);
          best_inleir = eval_pair.second;
        }
      }

      reset();
      execution_time = double(cv::getTickCount() - start_time) / double(cv::getTickFrequency()) * 1000;
      return true;
    }

    void reset() {
      data.clear();
      best_score = 0.0;
    };

    T* getBestModel() {
      return best_model;
    }

    const std::vector<Parameter*>& getBestInliers() {
      return best_inleir;
    }

    double getExecutionTime() {
      return execution_time;
    }

  private:
    int max_iterations;
    double threshold;
    std::vector<Parameter*> data;

    T* best_model;
    std::vector<Parameter*> best_inleir;
    double best_score;
    double execution_time;
  };
}

#endif //RANSAC2_H