#!/usr/bin/env python

import random
import csv
import rospy
from ras_arduino_msgs.msg import ADConverter
from math import *
from pylab import *
import threading

class SensorDesc():
    channel = ""
    number = 0
    t = 0
    min_range = 0.0
    max_range = 0.0
    
def loadSensorParams(sens):
    polLen = rospy.get_param('~sensor_pol_coeffs', 0)
    assert(polLen > 1)
    assert(len(sens)>0)
    coeffs = []
    for i in range(0, len(sens)):
        c = []
        for j in range(0, polLen):
            c += [rospy.get_param("~sensor{}_coeff{}".format(sens[i].number, j))]
        coeffs += [c]
    return coeffs
    
def loadSensorDesc():
    count = rospy.get_param('~sensor_max_count', 0)
    sens = []
    for i in range(1, count+1):
        t = rospy.get_param('~sensor{}_type'.format(i), 0)
        if(t > 0):
            assert(t == 1 or t == 2)
            sensor = SensorDesc()
            sensor.number = i
            sensor.channel = "ch{}".format(i)
            sensor.t = t
            sensor.max_range = rospy.get_param('~type{}_range_max'.format(t), 0.0)
            sensor.min_range = rospy.get_param('~type{}_range_min'.format(t), 0.0)
            sens += [sensor]
    return sens


class SensorCalib():
    noMes = 20
    dstLong = [0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]
    dstShort = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4]
    #dstLong = [0.2, 0.25, 0.3, 0.35]
    #dstShort = [0.1, 0.15, 0.2]
    adcPoly = frange(75.0, 515.0, 1.0)
    
    coeffs = []
    sensors = []
    sensorNumbers = []
    sensorNames = []
    lck = threading.Lock()
    measureCount = 0
    measuringEnabled = False
    curMeasurement = []
    curSenIdx = 0
    
        
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
        self.curMeasurement += [getattr(msg, self.sensors[self.curSenIdx].channel)]
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
        dstPoly = polyval(pol, self.adcPoly)
        #Plot everything.  
        pdatpen, = plot(adc, dst,'bx',
          label = "Measured".decode('utf-8'))
        plot(self.adcPoly, dstPoly,
              'r-',label = "Regression".decode('utf-8'), linewidth = 3)
        ylabel('Distance [m]'.decode('utf-8'))
        xlabel('ADC value [-]')
        title('Distance Sensor'.decode('utf-8'))
        grid(True)
        legend(loc=1)
        show()

    def calibSensor(self, index):
        self.curMeasurement = []
        dst = []
        print "Ready to calibrate IR sensor {}, type {}.\n".format(self.sensors[index].number, self.sensors[index].t)
        print "Press Enter to proceed."
        raw_input()
        if self.sensors[index].t==1:
            vals = self.dstShort
        else:
            vals = self.dstLong
        for d in vals:
            print "Place an object {} m from IR sensor {} and press Enter.\n".format(d, self.sensors[index].number)
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
            awriter.writerow(["sensor_pol_coeffs:", len(self.coeffs[0])])
            for i in range(0, len(self.sensors)):
                for j in range(0, len(self.coeffs[i])):
                    awriter.writerow(["sensor{}_coeff{}:".format(self.sensors[i].number, j), self.coeffs[i][j]])
                
    def calibAll(self):
        self.sensors = loadSensorDesc()
        yamlFile = rospy.get_param('~sensors_coeff_file', "")
        rospy.Subscriber("/arduino/adc", ADConverter, self.readAdc)
        for i in range(0, len(self.sensors)):
            self.calibSensor(i)
        self.saveYaml(yamlFile)

if __name__ == '__main__':
    rospy.init_node('ir_calibration')
    
    scalib = SensorCalib()
    scalib.calibAll()
    
