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
#include "costFunctions.hpp"
#include "processData.hpp"

int main(int argc, char** argv){

    std::string filename("../data/robotdata1.log");
    ProcessData processData(filename);

    google::InitGoogleLogging(argv[0]);

    ceres::Problem problem;
    
    double u[6];

    u[0]=0.00;
    u[1]=0.00;
    u[2]=0.001;
    u[3]=0.00;
    u[4]=0.001;
    u[5]=0.001;

    std::cout << "Initial" << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << "u " << i << ": " << u[i] << std::endl;
    }
    double Pij[6][6];

    Pij[0][0] = 0.0;
    Pij[0][1] = 10.0;
    Pij[0][2] = 12.0;
    Pij[0][3] = -10.0;
    Pij[0][4] = -12.0;
    Pij[0][5] = -11.0;

    Pij[1][0] = 10.0;
    Pij[1][1] = 0.0;
    Pij[1][2] = 15.0;
    Pij[1][3] = -5.0;
    Pij[1][4] = -15.0;
    Pij[1][5] = -10.0;

    Pij[2][0] = 12.0;
    Pij[2][1] = 15.0;
    Pij[2][2] = 0.0;
    Pij[2][3] = -8.0;
    Pij[2][4] = -6.0;
    Pij[2][5] = -3.0;

    Pij[3][0] = -10.0;
    Pij[3][1] = -5.0;
    Pij[3][2] = -8.0;
    Pij[3][3] = 0.0;
    Pij[3][4] = 10.0;
    Pij[3][5] = 5.0;

    Pij[4][0] = -12.0;
    Pij[4][1] = -15.0;
    Pij[4][2] = -6.0;
    Pij[4][3] = 10.0;
    Pij[4][4] = 0.0;
    Pij[4][5] = 11.0;


    Pij[5][0] = -11.0;
    Pij[5][1] = -10.0;
    Pij[5][2] = -3.0;
    Pij[5][3] = 5.0;
    Pij[5][4] = 11.0;
    Pij[5][5] = 0.0;

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            if (i==j)
            {
                continue;
            }

            ceres::CostFunction *costij = new ceres::AutoDiffCostFunction<ClassficationError, 1, 6>(
            new ClassficationError(10.0, Pij[i][j] + 15.0, i, j));

            problem.AddResidualBlock(costij, NULL, u);
        }
        

        ceres::CostFunction *magi = new ceres::AutoDiffCostFunction<MagError, 1, 6>(
        new MagError(10.0, i));

        problem.AddResidualBlock(magi, NULL, u);
    }

    ceres::CostFunction *atleastZero = new ceres::AutoDiffCostFunction<AtleastZeroError, 1, 6>(
            new AtleastZeroError(0.001, 6));

    problem.AddResidualBlock(atleastZero, NULL, u);

    ceres::CostFunction *atleastOne = new ceres::AutoDiffCostFunction<AtleastOneError, 1, 6>(
            new AtleastOneError(0.001, 6));

    problem.AddResidualBlock(atleastOne, NULL, u);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    options.use_inner_iterations = true;
    options.minimizer_progress_to_stdout = true;

    // Solve the problem and print the results
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "Final" << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << "u " << i << ": " << u[i] << std::endl;
    }

    return 0;

}