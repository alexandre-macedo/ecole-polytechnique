#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <array>
#include <memory>

namespace RANSAC {

    class Parameter {
    public:
        virtual ~Parameter() {};
    };

    template <int model_size>
    class Model {
    protected:
        std::array<std::shared_ptr<Parameter>, model_size> min_model_param;
        virtual double computeDistance(std::shared_ptr<Parameter> param) = 0;

    public:
        virtual void initialize(std::vector<std::shared_ptr<Parameter> > input_param) = 0;
        virtual std::pair<double, std::vector<std::shared_ptr<Parameter> > > evaluate(
                std::vector<std::shared_ptr<Parameter> > evaluate_param,
                double threshold) = 0;

        virtual std::array<std::shared_ptr<Parameter>, model_size> getModelParams(void) {
            return min_model_param;
        };
    };
}

#endif //MODEL_H
