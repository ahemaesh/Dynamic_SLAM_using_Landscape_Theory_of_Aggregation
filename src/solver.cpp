/*
 * Pose adjustmer using keypoint likelihoods from a single image
 * Author: Krishna Murthy
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains various cost function struct specifications
#include "processData.hpp"
#include "Classifier.hpp"
#include "dynamicSlam.hpp"
#include "visualize.hpp"

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);

    std::string filename("../data/robotdata1.log");
    ProcessData processData(filename);

    DynamicSLAM::Propensity propensity;

    auto correspondedScans = processData.getCorrespondedScans(0, 20);
    propensity.calculateDistances(correspondedScans);

    std::vector<bool> status(correspondedScans[0].size(), false);
    plotPoints(correspondedScans[0], status);

    std::vector<std::vector<double>> Pij;
    std::vector<double> Weight;

    propensity.calculatePropensity(Pij, Weight);

    std::cout << "Correspondences out of 180 : " << correspondedScans[0].size() << std::endl;

    std::vector<bool> init;
    std::vector<double> S(6, 1.0);
    std::vector<std::vector<double>> P(6, std::vector<double>(6, 0.0));

    init.emplace_back(true);
    init.emplace_back(false);
    init.emplace_back(false);
    init.emplace_back(true);
    init.emplace_back(true);
    init.emplace_back(false);

    P[0][0] = 0.0;
    P[0][1] = 10.0;
    P[0][2] = 12.0;
    P[0][3] = -10.0;
    P[0][4] = -12.0;
    P[0][5] = -11.0;

    P[1][0] = 10.0;
    P[1][1] = 0.0;
    P[1][2] = 15.0;
    P[1][3] = -5.0;
    P[1][4] = -15.0;
    P[1][5] = -10.0;

    P[2][0] = 12.0;
    P[2][1] = 15.0;
    P[2][2] = 0.0;
    P[2][3] = -8.0;
    P[2][4] = -6.0;
    P[2][5] = -3.0;

    P[3][0] = -10.0;
    P[3][1] = -5.0;
    P[3][2] = -8.0;
    P[3][3] = 0.0;
    P[3][4] = 20.0;
    P[3][5] = 5.0;

    P[4][0] = -12.0;
    P[4][1] = -15.0;
    P[4][2] = -6.0;
    P[4][3] = 20.0;
    P[4][4] = 0.0;
    P[4][5] = 11.0;

    P[5][0] = -11.0;
    P[5][1] = -10.0;
    P[5][2] = -3.0;
    P[5][3] = 5.0;
    P[5][4] = 11.0;
    P[5][5] = 0.0;

    Classifier classifier;

    auto ans = classifier.classify(P, S, init);

    std::cout << "Initial" << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "init " << i << ": " << init[i] << std::endl;
    }

    std::cout << "Final" << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << "ans " << i << ": " << ans[i] << std::endl;
    }

    return 0;

}