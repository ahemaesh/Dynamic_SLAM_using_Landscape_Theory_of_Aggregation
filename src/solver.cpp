
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

#include "processData.hpp"
#include "Classifier.hpp"
#include "Classifier2.hpp"
#include "dynamicSlam.hpp"
#include "visualize.hpp"
#include <ctime>
#include <cstdlib>

void randomInitialization(std::vector<bool> &state2);

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);

    std::string filename("../data/robotdata1.log");
    ProcessData processData(filename);

    auto correspondedScans = processData.getCorrespondedScans(100, 5);
    std::cout << "Correspondences out of 180 : " << correspondedScans[0].size() << std::endl;


    std::vector<bool> state(correspondedScans[0].size(), false);
    std::vector<std::vector<double>> Pij;
    std::vector<double> Weight;

    DynamicSLAM::Propensity propensity;
    propensity.calculateDistances(correspondedScans);
    propensity.calculatePropensity(Pij, Weight, state);

    randomInitialization(state);
    plotPoints(correspondedScans[0], state);

    Classifier2 classifier;
    classifier.classify(Pij, Weight, state);

    plotPoints(correspondedScans[0], state);

    return 0;
}

void randomInitialization(std::vector<bool> &state2)
{
    srand(time(nullptr));

    for (auto && i : state2)
    {
        i = rand() % 2;
    }
}
