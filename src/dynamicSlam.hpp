#include <vector>

#include "processData.hpp"

namespace DynamicSLAM{

	class Propensity{
		private:
			std::vector<std::vector<double> > distanceTranspose;
			std::vector<std::vector<double> > distances;
			std::vector<double> standardDeviations;
			double thersholdInit;

			void resizeVec(std::vector<std::vector<double> > &vec, const unsigned short ROWS, const unsigned short COLUMNS){
			    vec.resize(ROWS);
			    for( auto &it : vec ){
			        it.resize(COLUMNS);
			    }
			}
			
			void calculateStandardDeviation(std::vector<double> data, double &stdDev);

			static double calculateFeatureSum(std::vector<double> data1, std::vector<double> data2);
			
		
		public:
			Propensity()
			{
			    this->thersholdInit = 11.0;
			}  //Constructor
			
			~Propensity(){} //Destructor

			void
            calculatePropensity(std::vector<std::vector<double> > &propensityScore, std::vector<double> &playerWeight,
                                std::vector<bool> &init);

			void calculateDistances(std::vector<std::vector<Point> > &scans);
	};
}



