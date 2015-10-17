#include "detection/LinearClassifier.h"


my_float LinearClassifier::dotProduct(std::vector<my_float> in)
{
    assert(weights.size() == in.size());
    for(int i = 0; i < weights.size(); i++){
        in[i] = in[i] - avg[i];
    }
    my_float out = bias;
    for(int i = 0; i < weights.size(); i++){
        out += weights[i] * in[i];
    }
    return out;
}

void LinearClassifier::worker(const std::vector< std::vector<my_float> >& in, std::vector<my_float>& out, int start, int numTasks)
{    
    for (int j = start; j < start+numTasks; j++){
        out[j] = dotProduct(in[j]);
    }
}


std::vector <my_float> LinearClassifier::classify(std::vector< std::vector<my_float> > data, int threads)
{
    assert(threads > 0);
    clsResult.clear();
    int size = data.size();
    
    if (size == 1) {
        clsResult.push_back(dotProduct(data[0]));
    } else {
        int num_tasks = size < threads ? 1 : threads;
        int data_per_thread = size / num_tasks;
        int remaining = size;
        std::vector<std::thread> workers;
        clsResult.resize(size);
        int pos = 0;
        for (int i = 0; i < num_tasks; i++) {
            int num = i == num_tasks - 1 ? remaining : data_per_thread;
            workers.push_back(std::thread(std::bind(&LinearClassifier::worker, this, std::cref(data), std::ref(clsResult), pos, num)));
            pos += num;
            remaining -= num;
        }
        assert(remaining == 0);
        for (int i = 0; i < num_tasks; i++) {
            workers[i].join();
        }
    }
    return clsResult;    
}

std::vector <int> LinearClassifier::getPositive(std::vector< std::vector<my_float> > data, int threads)
{
    std::vector <int> posIdx;
    classify(data, threads);
    for(int i = 0; i < clsResult.size(); i++){
        if(clsResult[i]>0)
            posIdx.push_back(i);
    }
    return posIdx;
}

void LinearClassifier::initTestData(void)
{
    const int dataCount = 3;
    my_float df[][5] = {{1.0, 2.0, 3.0, 4.0, 5.0}, {-1.0, -1.1, 1.0, 2.0, -3.0}, {-1.0, 0.0, -0.1, 0.2, 8.0}};
    for(int i = 0; i < dataCount; i++){
        std::vector<my_float> d (df[i], df[i] + sizeof(df[i]) / sizeof(my_float) );
        testData.push_back(d);
    }
}

LinearClassifier::LinearClassifier(const char modelFile[])
{
    //this->testingEnabled = false;
    // TODO Load weights and the average image from a file.
    std::ifstream ifs(modelFile);
    assert(ifs.is_open());
    int dim;
    ifs >> dim;
    assert(dim>0);
    ifs >> this->bias;
    my_float ff;
    for( int i = 0; i < dim; i++){
        ifs >> ff;
        this->weights.push_back(ff);
    }
    for( int i = 0; i < dim; i++){
        ifs >> ff;
        this->avg.push_back(ff);
    }
    initTestData();
}

LinearClassifier::LinearClassifier(void)
{
    //this->testingEnabled = true;
    my_float wf[] = {2.0, -1.0, 1.2, 3.0, -2.3};
    std::vector<my_float> w (wf, wf + sizeof(wf) / sizeof(my_float) );
    my_float avf[] = {10.0, 4.8, 6.1, -2.5, 7.9};
    std::vector<my_float> av (avf, avf + sizeof(avf) / sizeof(my_float) );
    assert(w.size() == av.size());
    this->bias = 1.7;
    this->weights = w;
    this->avg = av;
    
    initTestData();
    assert(testData[0].size() == w.size());
}

bool LinearClassifier::testClassifier(void)
{
    //assert(testingEnabled == true);
    std::vector <my_float> out;
    out = classify(testData, 2);
    
    const int dataCount = 3;
    assert(out.size() == dataCount);
    my_float et = 1e-5;
    my_float expOut[] = {7.25+1.7, 18.05, -15.07};
    for(int i = 0; i<dataCount; i++){
        if(!(out[i] < (expOut[i]+et)) && (out[i] > (expOut[i]-et)) )
            return false;
    }
    return true;
}
