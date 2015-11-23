#include "detection/LinearClassifier.h"

class MultiClassifier{
	
public: 

	MultiClassifier(const char modelFile[]);
	
	std::vector <struct Classification> MVote(std::vector< std::vector<my_float> > data, int threads, int number_of_obj);
	
private:
	std::vector<my_float> avg;
	
	std::vector<std::vector<my_float>> weights;
	
	std::vector<my_float> bias;
	std::vector<int> voting_vector;

};
