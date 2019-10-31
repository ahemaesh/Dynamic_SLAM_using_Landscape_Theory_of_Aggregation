#include <cmath>
#include <iostream>
#include <vector>

#include "dynamicSlam.h"

namespace  DynamicSLAM{
	void Propensity::calculatePropensity(std::vector<float> &distances, std::vector<std::vector<float> > &propensityScore, std::vector<float> &playerSize){
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
}


int main(){
	//Avinash :  Main function is for testing
	DynamicSLAM::Propensity object_propensity = DynamicSLAM::Propensity();
	std::vector<float> nums1(5);
	std::vector<std::vector<float> > nums2;
	std::vector<float> nums3;
	object_propensity.calculatePropensity(nums1,nums2,nums3);
}
