from scipy import ndimage, misc
import matplotlib.pyplot as plt
import numpy as np
import csv
from svmutil import *

bbx = 30
bby = 30
nChannels = 3
svm_train_param = "-t 0 -c 10"
training_data_file = 'data/training_data.csv'
data_path_prefix = 'data/'
svm_model_out_file = 'models/cubeSvm.model'
lin_class_out_file = 'models/lin_class_cube_model.csv'

dimensions = bbx * bby * nChannels

def loadData(fileName):
    labels = []
    first = True
    with open(fileName, 'rb') as csvfile:
        areader = csv.reader(csvfile, delimiter='\t')
        for row in areader:
            img = misc.imread(data_path_prefix+row[0])
            labels += [int(row[1])]
            x0 = int(row[2])
            y0 = int(row[3])
            x1 = int(row[4])
            y1 = int(row[5])
            crop_img = img[y0: y1, x0: x1, 0:nChannels]
            cim = misc.imresize(crop_img, [bbx, bby])
            if first:
                data = np.array([np.transpose(cim, [2,0,1]).flatten()])
                first = False
            else:
                data = np.append(data, [np.transpose(cim, [2,0,1]).flatten()], axis = 0)
    return data, labels

def svmModel2Numpy(model, dataDimension):
    nsv = model.nSV[0] + model.nSV[1]
    sv = np.zeros((nsv, dataDimension))
    coefs = np.zeros(nsv)
    bias = -model.rho.contents.value
    for i in range(0, nsv):
        coefs[i] = model.sv_coef[0][i]
        for d in range(0, dataDimension):
            sv[i, d] = model.SV[i][d].value
    return sv, coefs, bias

def getWeights(sv, coefs):
    A = np.tile(coefs, (sv.shape[1], 1))
    W = np.sum(A.T * sv, axis = 0)
    return W

def saveLinClassModel(dim, bias, w, avg, fileName):
    with open(fileName, 'wb') as f:
        f.write('{} '.format(dim))
        f.write('{} '.format(bias))
        for x in w:
            f.write('{} '.format(x))
        for x in avg:
            f.write('{} '.format(x))
        f.write('\n')


trainData, trainLabels = loadData(training_data_file)
avg = np.sum(trainData, axis = 0)/float(len(trainLabels))
mavg = np.tile(avg, (len(trainLabels), 1))
trainData = trainData - mavg
trainData = trainData.tolist()
avg = avg.tolist()

#plt.imshow(img)
#plt.show()

# [l,a,b,e,l,s], [[feature],[vectors]]
prob = svm_problem(trainLabels,trainData)
param = svm_parameter(svm_train_param)
## training  the model
m = svm_train(prob, param)
svm_save_model(svm_model_out_file, m)
sv, coefs, bias = svmModel2Numpy(m, dimensions)
W = getWeights(sv, coefs)
saveLinClassModel(dimensions, bias, W, avg, lin_class_out_file)

#testing the model
p_label, p_acc, p_val = svm_predict(trainLabels,trainData, m)
#print p_label

#m = svm_load_model('cubeSvm.model')
