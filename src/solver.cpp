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

    auto correspondedScans = processData.getCorrespondedScans(100, 5);


    std::vector<bool> status(correspondedScans[0].size(), false);
    std::vector<std::vector<double>> Pij;
    std::vector<double> Weight;

    propensity.calculateDistances(correspondedScans);
    propensity.calculatePropensity(Pij, Weight, status);

    //plotPoints(correspondedScans[0], status);
    std::cout << "Correspondences out of 180 : " << correspondedScans[0].size() << std::endl;

    Classifier classifier;

    auto ans = classifier.classify(Pij, Weight, status);

    plotPoints(correspondedScans[0], ans);

    return 0;

}