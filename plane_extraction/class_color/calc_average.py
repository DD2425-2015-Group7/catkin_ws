from scipy import ndimage
from scipy import misc
import numpy as np
import os
import matplotlib.pyplot as plt
import csv

def saveAverage(avg, fileName):
    with open(fileName, 'wb') as f:
        for x in avg:
            for y in x:
                f.write('{} '.format(y))
            f.write('\n')

path = '/home/ras27/Documents/data/object'
#test_shape = misc.imread('bilder/' + '2015-10-19 14.03.10.jpg')
for i in range(1,17):
	k = 0
	test = np.zeros([1, 3])
	for filename in os.listdir(path+str(i)):
		#print(filename)
		k += 1
		face = misc.face()

		face = misc.imread(path + str(i) +'/'+ filename)

		face = np.mean(face, axis = 0)
		face = np.mean(face, axis = 0)
		#print((face))
	#print((face[250][500]))
		test += face.astype(int)


		#print((face.dtype))
	
	leng = np.sqrt((test[0][0]**2 + test[0][1]**2 + test[0][2]**2))
	print(leng)
	test = (test /leng)
	#test = test.astype(int)
	print(test)
	outfile = 'averageAll' + str(i) + '.csv'
	saveAverage(test, outfile)
	#print((test[250][500]))
