#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "dynamicSlam.hpp"

namespace  DynamicSLAM{
	void calculatePropensity(std::vector<std::vector<double> > &propensityScore, std::vector<double> &playerWeight){
		// vector initialization of fixed size
		int timeStep = distances.size();
		int vecSize = distances[0].size();  
		playerWeight.resize(vecSize);
		resizeVec(propensityScore,vecSize,vecSize);
		
		for(int i=0;i<propensityScore.size();i++){
			for(int j=0;j<propensityScore[0].size();j++){
				propensityScore[i][j] = (1/timeStep)*(abs(featureSum[i] - featureSum[j]));
			}
			playerWeight[i] = 1/standardDeviations[i];
		}
	}

	void Propensity::calculateDistances(std::vector<std::vector<Point> > &scans){

		Eigen::VectorXd point;
		sigma_inv = Eigen::MatrixXd::Identity(2,2);
		resizeVec(distances,scans.size()-1,scans[0].size());
		resizeVec(distanceTranspose,scans[0].size(),scans.size()-1)
		standardDeviations.resize(scans[0].size())
		featureSum.resize(scans[0].size())

		double stdDev = 0;
		double sum = 0;


		for(int scan=1; scan<scans.size(); scan++){
			for(int feature=0; feature<scans[0].size();feature++){
				point = abs(scans[0][feature]-scans[scan][feature]);
				distances[scan-1][feature] = sqrt(point.transpose()*sigma_inv*point);
				distanceTranspose[feature].push_back(distances[scan-1][feature]);
			}
		}

		for(int i=0;i<distanceTranspose.size();i++){
			Propensity::calculateStandardDeviation(distanceTranspose[i], stdDev, sum);
			standardDeviations[i] = stdDev;
			featureSum[i] = sum;
		}

	}

	void Propensity::calculateStandardDeviation(std::vector<double> data, double &stdDev, double &sum){
		int data_length = data.size();
		double variance = 0;
	    double mean = 0;

	    stdDev = 0;
	    sum = 0;

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

int main(){
	//Avinash :  Main function is for testing
	DynamicSLAM::Propensity object_propensity = DynamicSLAM::Propensity();
	std::vector<double> nums1(5);
	std::vector<std::vector<double> > nums2;
	std::vector<double> nums3;
	object_propensity.calculatePropensity(nums1,nums2,nums3);
}
