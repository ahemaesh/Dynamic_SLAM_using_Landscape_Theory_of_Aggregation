//
// Created by parv on 11/4/19.
//

#ifndef D_SLAM_CLASSIFIER2_HPP
#define D_SLAM_CLASSIFIER2_HPP


#include <vector>

class Classifier2
{
public:
    Classifier2();
    void classify(const std::vector<std::vector<double>>& P, const std::vector<double>& S, std::vector<bool> &state);

private:
    int numIteration;
    double computeCostChange(const std::vector<std::vector<double>> &P, const std::vector<double> &S,
                             int featureIndex, std::vector<bool> &state);

    void correctClassAssignment(const std::vector<double> &S, std::vector<bool> &state);
};


#endif //D_SLAM_CLASSIFIER2_HPP
