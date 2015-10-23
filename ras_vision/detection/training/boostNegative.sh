#!/bin/bash
DATA_PREFIX="~/bigData/ras2015/vision"
SESSION_ID=5

DATA_PREFIX="${DATA_PREFIX/#\~/$HOME}"
cd $(rospack find detection)
echo "$DATA_PREFIX/rendered/"
ln -s "$DATA_PREFIX/rendered/" data/rendered
ln -s "$DATA_PREFIX/negative_train/" data/negative_train
ln -s "$DATA_PREFIX/negative_boost/" data/negative_boost
roslaunch detection bag_detect.launch bag:="$DATA_PREFIX/negative1-2015-10-20.bag" start:=301 duration:=100
roslaunch detection boost_sample.launch session_id:=$SESSION_ID bag:="$DATA_PREFIX/negative1-2015-10-20.bag"

cp data/training_data0.csv data/training_data.csv
cat "data/negative_boost/neg_boost$SESSION_ID.csv" >> data/negative_boost/neg_boost.csv
cat data/negative_boost/neg_boost.csv >> data/training_data.csv

python training/trainLinClass.py
cp "models/lin_class_cube_model.csv" "models/lin_class_cube_model$SESSION_ID.csv"

rm data/negative_boost data/negative_train data/rendered
