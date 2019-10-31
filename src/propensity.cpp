#include <iostream>
#include <vector>

#include "propensity.h"

namespace  DynamicSLAM{
	namespace Propensity{
		void Propensity::calculatePropensity(std::vector<float> &pointValues){
			int vecSize = pointValues.size();
			std::vector<std::vector<float> > Pij(vecSize,std::vector<float>(vecSize));
			
			for(int i=0;i<Pij.size();i++){
				for(int j=0;j<Pij[0].size();j++){
					std::cout << "\t" << Pij[i][j];
				}
				std::cout << std::endl;
			}
		}
	}
}


int main(){
	DynamicSLAM::Propensity::Propensity object_propensity = DynamicSLAM::Propensity::Propensity();
	std::vector<float> nums(5);
	object_propensity.calculatePropensity(nums);
}
