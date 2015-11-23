#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <assert.h>
#include <cmath>

typedef double my_float;

struct Classification{
    int index;
	int obj_type;
    my_float prob;
};

class LinearClassifier{

public:
    LinearClassifier(const char modelFile[]);
    LinearClassifier(void);
    LinearClassifier( std::vector<my_float> w, std::vector<my_float> average, my_float b);

    std::vector <struct Classification> getPositive(std::vector< std::vector<my_float> > data, int threads);
	std::vector <my_float> classify(std::vector< std::vector<my_float> > data, int threads);     
	bool testClassifier(void);
    
private:
    my_float dotProduct(std::vector<my_float> in);
    void worker(const std::vector< std::vector<my_float> >& in, /*std::vector<std::vector<my_float>>*/std::vector<my_float>& out, int start, int numTasks);

    void initTestData(void);
    
    std::vector <my_float> clsResult; //std::vector<std::vector<my_float> > clsResults 

    std::vector<my_float> avg; //std::vector<std::vector<my_float> > avg
    std::vector<my_float> weights; //std::vector<std::vector<my_float>> weights
    my_float bias; //std::vector<my_float>
    
    //bool testingEnabled;
    std::vector< std::vector<my_float> > testData;
};
