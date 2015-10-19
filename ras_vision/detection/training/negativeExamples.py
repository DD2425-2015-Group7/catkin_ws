import csv
import random

fileName = 'negative_train.csv'
fileNumbers = range(0,6)
boundingBoxesPerImage = 20
width = 320
height = 240
scales = [320, 240, 160, 120, 80, 60]
windowSizeX = 30
windowSizeY = 30
stepX = 3
stepY = 3

boundingBoxes = []
random.seed()

for s in range(0, len(scales), 2):
    for y in range(windowSizeY, scales[s+1], stepY):
        for x in range(windowSizeX, scales[s], stepX):
            ksx = width/scales[s];
            ksy = height/scales[s+1];
            bbt = []
            bbt += [ksx*(x-windowSizeX) + 1]
            bbt += [ksy*(y-windowSizeY) + 1]
            bbt += [ksx*x]
            bbt += [ksy*y]
            boundingBoxes += [bbt]


with open(fileName, 'wb') as csvfile:
    awriter = csv.writer(csvfile, delimiter='\t')
    for n in fileNumbers:
        for i in range(0, boundingBoxesPerImage):
            bb = boundingBoxes[random.randrange(0, len(boundingBoxes))]
            awriter.writerow(["negative_train/neg{}.png".format(n), -1, bb[0], bb[1], bb[2], bb[3]])
