#include <vector>

namespace DynamicSLAM{

	class Propensity{
		private:
			void resizeVec(std::vector<std::vector<double> > &vec, const unsigned short ROWS, const unsigned short COLUMNS){
			    vec.resize(ROWS);
			    for( auto &it : vec ){
			        it.resize(COLUMNS);
			    }
			}
			st
			
		
		public:
			Propensity(){}  //Constructor
			
			~Propensity(){} //Destructor


			void calculatePropensity(std::vector<std::vector<double> > &landmarks, std::vector<double> &distances, std::vector<std::vector<double> > &propensityScore, std::vector<double> &playerWeight);

	};


	class Distances{
		public:
			void calculateDistances(std::vector<std::vector<double> > &landmarks, std::vector<std::vector<double> > &distances);

	}


}



