#include "detection/MultiClassClassifier.h"
//int model_iter = 11; //FOR NOW IF WE ADD OBJECTS WE NEED TO CHANGE THIS......
MultiClassifier::MultiClassifier(const char modelFile[]){
    //initialize avg, weights, bias,
    std::ifstream ifs(modelFile);
    assert(ifs.is_open());
    try {

        int dim;
        my_float b;
        my_float ff;
        bool f = true;
        ifs >> dim;
        //ifs >> dim;
        assert(dim>0);
        int j = 0;
        for(int i = 0; i<66; i++){
            std::vector<my_float> temp;
            this->weights.push_back(temp);
            ifs>>b;
            this->bias.push_back(b);
            for(int j = 0; j<dim; j++){
                ifs>>ff;
                this->weights[i].push_back(ff);


            }
            if(f){
                for(int j = 0; j<66; j++){
                    ifs>>ff;

                    this->avg.push_back(ff);
                }
                f = false;
            }
            j+=1;
        }
    } catch (...){

        std::cerr << "error in load" << std::endl;

    }
    /*std::cerr << "finished loading model" << std::endl;
    std::cerr << "weight size: " << weights.size() << ", " << weights[1].size()<<std::endl;
    std::cerr << "average size: " << avg.size() << std::endl;
    std::cerr << "dim: " << dim << std::endl;*/
}

std::vector<struct Classification> MultiClassifier::MVote(std::vector<std::vector<my_float>> data, int threads, int number_of_obj){
   //perform the voting
    //1. create a linear classifier for each model
    //2. run the LC and get the resulting pos/neg
    //3. Count the "win" for the corresponding class in voting_vector
    //4. return the class with biggest vote. if same then take class with lower index
    bool first = true;
    int counter = 0;
    int mi = 0;
    int mj = 1;
    std::vector<std::vector<int>> vote; //[n_class, bb_size]
    std::vector<std::vector<my_float>> prob_init;
    int part_vote;
    //---------------------------------------Do voting here ---------------------------------------
    // we have 66 models=> 66 sets of weights, 12*(12-1)/2, first 11 models is svm(1,i)|i is [2,12]


    //ERROR HERE ------------------------------------------------
    for(int i = 0; i<weights.size();i++){

        if(first || counter >= number_of_obj-mi){
            //change model handle

           // std::cerr <<  "count: " << counter << std::endl;
            mi += 1;
            mj = mi+1;
           // std::cerr << " i: " << mi <<", j: " << mj << std::endl;
            counter = 0;
           // std::cerr <<  "index : " << i << std::endl;
        }

        //access the parameters for the ith model, initialize it and classify the data wiath the model
        std::vector<my_float> w=weights[i];
        my_float b = bias[i];
       // std::cerr << "in vote" << std::endl;

        std::vector<my_float> average = avg;
        LinearClassifier linclass = LinearClassifier(w,average,b);
        std::vector<my_float> result = linclass.classify(data,threads);
        prob_init.push_back(result);
        //specify the second dimension (bounding box) dimension.
        part_vote = result.size();


        std::vector<int> till(part_vote,0);
        //  If the matrix has not yet been fully initialized, initialize it
        if(first){
            //std::cerr<< weights.size() << std::endl;
            for(int b = 0; b<number_of_obj; b++){

                    vote.push_back(till);

            }
            first = false;
        }

        //perform the main voting of the result depending on which model we are in.

        for(int k = 0; k<result.size();k++){
            //std::cerr << "vote size: " << vote.size() << ", " << vote[0].size();


            if(result[k]>0){
                //std::cerr << "error start" << std::endl;

                vote[mi-1][k]+=1;
                //std::cerr << "error end" << std::endl;
            }else{
                vote[mj-1][k]+=1;
            }
        }

        //change to the next model to compare with. And add to the counter
        mj += 1;
        counter += 1;


    }




   // std::cerr << "after vote/before publishing" << std::endl;
    //--------------------------------------Find Maximum Vote-------------------------------------
    //we want to find the maximum class in each bounding box.

    std::vector<int> major_vote_r;
    std::vector <struct Classification> classvector;
    int flag;
    flag = 0;
    std::vector<double> prob(part_vote,0);
    for(int b = 0; b<part_vote; b++){
        for(int n = 0; n<number_of_obj; n++){
            if(vote[n][b]>flag){
                flag = vote[n][b];

                prob[b] = 1/(1+exp(-prob_init[n][b]));
                if(prob[b]>0.99){
                    struct Classification klass;
                    klass.prob = prob[b];
                    klass.index = b;
                    klass.obj_type = n+1;
                    classvector.push_back(klass);
                    //major_vote_r.push_back(n+1); //if the probability is higher than 70% that )
                }
            }
        }
        flag = 0;


        //classvector.push_back(klass);
    }
    std::cerr << "size" << classvector.size() << std::endl;
    return classvector;

    //std::cerr << "error in the publishing" << std::endl;


}
