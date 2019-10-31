#include <cmath>
#include <iostream>
#include <vector>

#include "dynamicSlam.h"

namespace  DynamicSLAM{
	void Propensity::calculatePropensity(std::vector<double> &distances, std::vector<std::vector<double> > &propensityScore, std::vector<double> &playerWeight){
		// vector initialization of fixed size
		int vecSize = distances.size();  
		playerSize.resize(vecSize);
		resizeVec(propensityScore,vecSize,vecSize);
		

		for(int i=0;i<propensityScore.size();i++){
			for(int j=0;j<propensityScore[0].size();j++){
				// propensityScore[i][j] = std::norm(distances[i]-distances[j]);
				std::cout << "\t" << propensityScore[i][j];
			}
			std::cout << std::endl;
		}
	}

	void Distances::calculateDistances(std::vector<std::vector<std::point> > &landmarks, std::vector<std::vector<double> > &distances){

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
