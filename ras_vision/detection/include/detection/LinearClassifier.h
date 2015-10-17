#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <assert.h>

typedef double my_float;

class LinearClassifier{

public:
    LinearClassifier(const char modelFile[]);
    LinearClassifier(void);
    
    std::vector <int> getPositive(std::vector< std::vector<my_float> > data, int threads);
    bool testClassifier(void);
    
private:
    my_float dotProduct(std::vector<my_float> in);
    void worker(const std::vector< std::vector<my_float> >& in, std::vector<my_float>& out, int start, int numTasks);
    std::vector <my_float> classify(std::vector< std::vector<my_float> > data, int threads); 
    void initTestData(void);
    
    std::vector <my_float> clsResult;

    std::vector<my_float> avg;
    std::vector<my_float> weights;
    my_float bias;
    
    //bool testingEnabled;
    std::vector< std::vector<my_float> > testData;
};
