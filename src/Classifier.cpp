//
// Created by parv on 10/31/19.
//

#include "Classifier.hpp"

Classifier::Classifier()
{
    this->class1 = 0.0;
    this->class2 = 0.001;

    this->scaleP = 10.0;
    this->scaleDiscrete = 10.0;
    this->scaleAtleastOne = 0.001;

    this->boundary = (this->class1 + this->class2)/2.0;
}

std::vector<bool> Classifier::classify(std::vector<std::vector<double>> P, std::vector<double> S, std::vector<bool> init)
{
    ceres::Problem problem;

    auto *u = new double[180];

    for(int i = 0; i < 180; i++)
    {
        if ( i < S.size() && init[i])
        {
            u[i] = this->class2;
        }
        else
        {
            u[i] = this->class1;
        }
    }

    addCosts(P, S, problem, u);
    optimize(problem);

    std::vector<bool> ans;

    for (int i = 0; i < S.size(); i++)
    {
        if (abs(u[i] - this->class1) <= abs(u[i] - this->class2))
        {
            ans.emplace_back(false);
        }
        else
        {
            ans.emplace_back(true);
        }
        std::cout << "u " << i << " :" << u[i] << std::endl;
    }

//    for (int i = 0; i < 180; i++)
//    {
//        std::cout << "u " << i << " :" << u[i] << std::endl;
//    }
    return ans;
}

void Classifier::optimize(ceres::Problem &problem) const {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    options.use_inner_iterations = true;
    options.minimizer_progress_to_stdout = false;

    // Solve the problem and print the results
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

void
Classifier::addCosts(const std::vector<std::vector<double>> &P, const std::vector<double> &S, ceres::Problem &problem,
                     double *u)
{
    int dim = S.size();
    auto minij = 999999999999999.9;

    for (int i = 0; i < dim; i++)
    {
        minij = std::min(minij, *std::min_element(P[i].begin(), P[i].end()));
    }

    for(int i = 0; i < dim; ++i)
    {
        for (int j = 0; j < dim; ++j)
        {
            if (i==j)
            {
                continue;
            }

            ceres::CostFunction *costij = new ceres::AutoDiffCostFunction<ClassficationError, 1, 180>(
                    new ClassficationError(this->scaleP * S[j] / S[i], P[i][j] - minij, i, j));

            problem.AddResidualBlock(costij, NULL, u);
        }


        ceres::CostFunction *magi = new ceres::AutoDiffCostFunction<MagError, 1, 180>(
                new MagError(this->scaleDiscrete, i, this->class1, this->class2));

        problem.AddResidualBlock(magi, NULL, u);
    }

    ceres::CostFunction *atleastOne = new ceres::AutoDiffCostFunction<AtleastOneError, 1, 180>(
            new AtleastOneError(this->scaleAtleastOne, dim));

    problem.AddResidualBlock(atleastOne, NULL, u);
}

