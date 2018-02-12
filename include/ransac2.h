#ifndef RANSAC2_H
#define RANSAC2_H

#include <vector>
#include <array>

namespace RANSAC {
  class Parameter {
  public:
    virtual ~Parameter() {}
  };

  template <int modelSize> class Model {
  public:
    virtual void initialize(const std::vector<Parameter*>& input_param) = 0;
    virtual std::pair<double, std::vector<Parameter*> > evaluate(
        std::vector<Parameter*> input_param, double threshold) = 0;

    virtual std::array<Parameter*, modelSize> getModelParameters() {
      return min_model_param;
    };

  protected:
    std::array<Parameter*, modelSize> min_model_param;

    virtual double computeDistance(Parameter* parameter) = 0;
  };
  
  template <class T, int modelSize> class Estimator {
  public:
    void initialize(double threshold, int max_iterations = 1000) {
      this->threshold = threshold;
      this->max_iterations = max_iterations;
    }

    bool estimate(const std::vector<Parameter*>& data) {
      if (data.size() <= modelSize) {
        std::cout << "Insuficient data to compute best model." << std::endl;
        execution_time = -1;
        return false;
      }

      int64 startTime = cv::getTickCount();
      this->data = data;
      std::uniform_int_distribution<int> uniDist(0, int(this->data.size() - 1));
      
      for (int i = 0; i < max_iterations; i++) {
        std::vector<Parameter*> random_samples(modelSize);
        std::vector<Parameter*> remainderSamples = this->data;
        std::random_device seed;
        std::mt19937 g(seed());
        std::shuffle(remainderSamples.begin(), remainderSamples.end(), g);
        std::copy(
            remainderSamples.begin(), remainderSamples.begin() + modelSize, random_samples.begin());
        remainderSamples.erase(remainderSamples.begin(), remainderSamples.begin() + modelSize);

        T randomModel(random_samples);
        std::pair<double, std::vector<Parameter*> > eval_pair = 
            randomModel.evaluate(remainderSamples, threshold);
        double score = eval_pair.first;

        if (score > best_score) {
          best_score = score;
          best_model = new T(random_samples);
          best_inleir = eval_pair.second;
        }
      }

      reset();
      execution_time = double(cv::getTickCount() - startTime) / double(cv::getTickFrequency()) * 1000;
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