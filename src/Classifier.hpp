//
// Created by parv on 10/31/19.
//

#ifndef D_SLAM_CLASSIFIER_HPP
#define D_SLAM_CLASSIFIER_HPP

#include "costFunctions.hpp"

class Classifier
{
public:
    Classifier();

    std::vector<bool> classify(std::vector<std::vector<double>> P, std::vector<double> S, std::vector<bool> init);

private:
    double scaleP;
    double scaleDiscrete;
    double scaleAtleastOne;
    double scaleAtleastZero;
    double class1;
    double class2;
    double boundary;

    void addCosts(const std::vector<std::vector<double>> &P, const std::vector<double> &S, ceres::Problem &problem,
             double *u);

    void optimize(ceres::Problem &problem) const;
};


#endif //D_SLAM_CLASSIFIER_HPP
