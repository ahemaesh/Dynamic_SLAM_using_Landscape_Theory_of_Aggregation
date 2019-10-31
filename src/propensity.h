#include <vector>

namespace DynamicSLAM{
	namespace Propensity{
		class Propensity{
			private:
				//place holder
				static const int i = 144; 
				
			
			public:
				Propensity(){
					// Initialize variables
				}
				
				~Propensity(){
					// Destroy variables/clear memory
				}


				void calculatePropensity(std::vector<float> &pointValues);

		};
	}
}



