#include <vector>

namespace DynamicSLAM{
	namespace Propensity{
		class Propensity{
			private:
				void resizeVec(std::vector<std::vector<float> > &vec, const unsigned short ROWS, const unsigned short COLUMNS){
				    vec.resize(ROWS);
				    for( auto &it : vec ){
				        it.resize(COLUMNS);
				    }
				}
				
			
			public:
				Propensity(){
					// Initialize variables
				}
				
				~Propensity(){
					// Destroy variables/clear memory
				}


				void calculatePropensity(std::vector<float> &distances, std::vector<std::vector<float> > &propensityScore, std::vector<float> &playerSize);

		};
	}
}



