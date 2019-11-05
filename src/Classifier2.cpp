//
// Created by parv on 11/4/19.
//

#include "Classifier2.hpp"
#include <iostream>

Classifier2::Classifier2()
{
    this->numIteration = 5;
}

void Classifier2::classify(const std::vector<std::vector<double>> &P, const std::vector<double> &S,
                           std::vector<bool> &state)
{
    for (int i = 0; i < this->numIteration; ++i)
    {
        int changeCount = 0;
        for (size_t j = 0; j < state.size(); ++j)
        {
            double cost = computeCostChange(P, S, j, state);
            if (cost < 0.0)
            {
                state[j] = !state[j];
                changeCount++;
            }
        }

        std::cout << "Optimizing: Total Variables Changed = " << changeCount << std::endl;
        if (changeCount == 0)
        {
            break;
        }
    }

}

double Classifier2::computeCostChange(const std::vector<std::vector<double>> &P, const std::vector<double> &S,
                                      int featureIndex, std::vector<bool> &state)
{
    bool updatedClass = !state[featureIndex];
    double cost  = 0;

    for (size_t j = 0; j < state.size(); ++j)
    {
        if (j != featureIndex)
        {
            if (updatedClass == state[j])
            {
                cost += -((S[j]*P[featureIndex][j])/S[featureIndex]) - ((S[featureIndex]*P[j][featureIndex])/S[j]);
            }
            else
            {
                cost += ((S[j]*P[featureIndex][j])/S[featureIndex]) + ((S[featureIndex]*P[j][featureIndex])/S[j]);
            }
        }
    }

}