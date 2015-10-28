#!/usr/bin/env python

import random
import csv
import rospy
import rosbag
from ras_arduino_msgs.msg import ADConverter
from math import *
from pylab import *
import threading


class SensorCalib():
    yamlFile = ""
    coeffs = []
    sensorNumbers = []
    sensorNames = []
    lck = threading.Lock()

    measureCount = 0
    noMes = 20
    measuringEnabled = False
    curMeasurement = []
    curSenIdx = 0
    #dstLong = [0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]
    dstLong = [0.2, 0.25, 0.3, 0.35]
    #dstShort = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4]
    dstShort = [0.1, 0.15, 0.2]
        
    def readAdc(self, msg):
        self.lck.acquire()
        if not self.measuringEnabled:
            self.lck.release()
            return
        if self.measureCount >= self.noMes:
            self.measuringEnabled = False
            self.lck.release()
            return
        self.measureCount += 1
        self.curMeasurement += [getattr(msg, self.sensorNames[self.curSenIdx])]
        self.lck.release()
        
    def wait4data(self):
        rate = rospy.Rate(10)
        while True:
            rate.sleep()
            self.lck.acquire()
            if not self.measuringEnabled:
                self.lck.release()
                return
            self.lck.release()
            
    def showResult(self, pol, adc, dst):
        adcPoly = frange(75.0, 515.0, 1.0)
        dstPoly = polyval(pol, adcPoly)
          
        #Plot everything.  
        pdatpen, = plot(adc, dst,'bx',
          label = "Measured".decode('utf-8'))
        plot(adcPoly, dstPoly,
              'r-',label = "Regression".decode('utf-8'), linewidth = 3)
        ylabel('Distance [m]'.decode('utf-8'))
        xlabel('ADC value [-]')
        title('Distance Sensor'.decode('utf-8'))
        grid(True)
        legend(loc=1)
        show()

    def calibSensor(self, index):
        n = -1
        self.curMeasurement = []
        dst = []
        while n!=0 and n!=1:
            print "Ready to calibrate IR sensor {}.\n".format(self.sensorNumbers[index])
            print "Choose: 0 - short range, 1 - long range:"
            try:
                n = int(raw_input())
            except ValueError:
                print "Not a number, please try again.\n"
                continue
        if n==0:
            vals = self.dstShort
        else:
            vals = self.dstLong
        for d in vals:
            print "Place an object {} m from IR sensor {} and press Enter.\n".format(d, self.sensorNumbers[index])
            raw_input()
            self.lck.acquire()
            self.measuringEnabled = True
            self.measureCount = 0
            self.curSenIdx = index
            self.lck.release()
            self.wait4data()
            for i in range(0, self.noMes): 
                dst += [d]
        pol = polyfit(self.curMeasurement, dst, 3)
        self.coeffs += [pol]
        self.showResult(pol, self.curMeasurement, dst)


    def saveYaml(self, fileName):
        with open(fileName, 'wb') as csvfile:
            awriter = csv.writer(csvfile, delimiter=' ')
            assert(len(self.coeffs)>=1)
            awriter.writerow(["ir_sensor_pol_coeffs:", len(self.coeffs[0])])
            for i in range(0, len(self.sensorNumbers)):
                for j in range(0, len(self.coeffs[i])):
                    awriter.writerow(["ir_sensor{}_coeff{}:".format(self.sensorNumbers[i], j), self.coeffs[i][j]])

    def loadParams(self):
        count = rospy.get_param('~ir_sensor_max_count', 0)
        self.yamlFile = rospy.get_param('~ir_sensors_coeff_file', "")
        for i in range(1, count+1):
            if(rospy.get_param('~ir_sensor{}_enabled'.format(i), False) == True):
                self.sensorNumbers += [i]
                self.sensorNames += ["ch{}".format(i)]
                
    def calibAll(self):
        self.loadParams()
        rospy.Subscriber("/arduino/adc", ADConverter, self.readAdc)
        for i in range(0, len(self.sensorNumbers)):
            self.calibSensor(i)
        self.saveYaml(self.yamlFile)

if __name__ == '__main__':
    rospy.init_node('ir_calibration')
    
    scalib = SensorCalib()
    scalib.calibAll()
    
