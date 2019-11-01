#include <vector>

#include "processData.hpp"

namespace DynamicSLAM{

	class Propensity{
		private:
			std::vector<std::vector<double> > distanceTranspose;
			std::vector<std::vector<double> > distances;
			std::vector<double> standardDeviations;	

			void resizeVec(std::vector<std::vector<double> > &vec, const unsigned short ROWS, const unsigned short COLUMNS){
			    vec.resize(ROWS);
			    for( auto &it : vec ){
			        it.resize(COLUMNS);
			    }
			}
			
			void calculateStandardDeviation(std::vector<double> data, double &stdDev);

			double calculateFeatureSum(std::vector<double> data1, std::vector<double> data2);
			
		
		public:
			Propensity(){}  //Constructor
			
			~Propensity(){} //Destructor

			void calculatePropensity(std::vector<std::vector<double> > &propensityScore, std::vector<double> &playerWeight);

			void calculateDistances(std::vector<std::vector<Point> > &scans);
	};
}



