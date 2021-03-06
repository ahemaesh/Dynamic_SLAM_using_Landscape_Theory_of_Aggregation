#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <numeric>


#include "dynamicSlam.hpp"

namespace  DynamicSLAM{
	void Propensity::calculatePropensity(std::vector<std::vector<double> > &propensityScore,
                                         std::vector<double> &playerWeight,
                                         std::vector<bool> &init) {
		// vector initialization of fixed size
		int timeStep = distances.size();
		int vecSize = distances[0].size();  
		playerWeight.resize(vecSize);
		resizeVec(propensityScore,vecSize,vecSize);
		
		for(int i=0;i<propensityScore.size();i++){
			for(int j=0;j<propensityScore[0].size();j++){
				propensityScore[i][j] = Propensity::calculateFeatureSum(distanceTranspose[i],distanceTranspose[j]);

			}
			playerWeight[i] = 1/standardDeviations[i];
		}

        for (size_t i = 0; i < distanceTranspose.size(); i++)
        {
            double avg = std::accumulate(distanceTranspose[i].begin(), distanceTranspose[i].end(), 0.0)/double(distanceTranspose[i].size());

            init[i] =(avg > this->thersholdInit);
        }
	}

	double Propensity::calculateFeatureSum(std::vector<double> data1, std::vector<double> data2){
		double sum = 0;
		for(int i=0;i<int(data1.size());i++){
			sum += abs(data1[i]-data2[i]);
		}

		return (1.0 - (sum/double(data1.size())));
	}

	void Propensity::calculateDistances(std::vector<std::vector<Point> > &scans){
		Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Identity(2,2);
		resizeVec(distances,scans.size()-1,scans[0].size());
		resizeVec(distanceTranspose,scans[0].size(),scans.size()-1);
		standardDeviations.resize(scans[0].size());
		
		double stdDev = 0;

		for(int scan=1; scan<scans.size(); scan++){
			for(int feature=0; feature<scans[0].size();feature++){
				Eigen::Vector2d  point1(scans[0][feature].x,scans[0][feature].y);
				Eigen::Vector2d  point2(scans[scan][feature].x,scans[scan][feature].y);
				//std::cout << "point1:" << point1 << std::endl;
                //std::cout << "point2:" << point2 << std::endl;
                Eigen::Vector2d point = point1-point2;
                //std::cout << "point:" << point << std::endl;
                distances[scan-1][feature] = sqrt(point.transpose()*sigma_inv*point);
				distanceTranspose[feature][scan-1]=(distances[scan-1][feature]);
			}
		}

		for(int i=0;i<distanceTranspose.size();i++){
			Propensity::calculateStandardDeviation(distanceTranspose[i], stdDev);
			standardDeviations[i] = stdDev;
		}
	}

	void Propensity::calculateStandardDeviation(std::vector<double> data, double &stdDev){
		int data_length = data.size();
		double variance = 0;
	    double mean = 0;
	    double sum = 0;

	    stdDev = 0;
	    for(int i=0; i<data_length;i++){
	    	sum += data[i];
	    }
	    mean = sum/data_length;

	    for(int i=0; i<data_length;i++){
	    	variance += pow((data[i]-mean),2);
	    }
	    variance /= data_length;
	    stdDev = sqrt(variance);

	    return ;
	}
}

// int main(){
// // 	//Avinash :  Main function is for testing
// // 	DynamicSLAM::Propensity object_propensity = DynamicSLAM::Propensity();
// // 	std::vector<double> nums1(5);
// // 	std::vector<std::vector<double> > nums2;
// // 	std::vector<double> nums3;
// // 	object_propensity.calculatePropensity(nums1,nums2,nums3);
//  	return 0;
//  }
