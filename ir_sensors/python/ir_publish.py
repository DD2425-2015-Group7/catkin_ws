#!/usr/bin/env python

import random
import csv
import rospy
from ras_arduino_msgs.msg import ADConverter
from sensor_msgs.msg import Range
from ir_sensors.msg import RangeArray
from math import *
from pylab import *
from ir_calibration import SensorDesc, loadSensorDesc, loadSensorParams


class SensorPublish():

    coeffs = []
    sensors = []
    msg = RangeArray()
    
    def __init__(self):
        self.irpub = rospy.Publisher("/ir_publish/sensors", RangeArray, queue_size = 5)
        self.sensors = loadSensorDesc()
        self.coeffs = loadSensorParams(self.sensors)
        self.prepareArray()
        rospy.Subscriber("/arduino/adc", ADConverter, self.readAdc)
    
    def prepareArray(self):
        for i in range(0, len(self.sensors)):
            r = Range()
            r.header.frame_id = "ir{}_link".format(i + 1)
            r.min_range = self.sensors[i].min_range
            r.max_range = self.sensors[i].max_range
            r.radiation_type = r.INFRARED
            self.msg.array += [r]
        
    def readAdc(self, msg):
        self.msg.header.stamp = rospy.Time.now()
        for i in range(0, len(self.sensors)):
            m = getattr(msg, self.sensors[i].channel)
            self.msg.array[i].range = polyval(self.coeffs[i], m)
            self.msg.array[i].header.stamp = self.msg.header.stamp
        self.irpub.publish(self.msg)


if __name__ == '__main__':
    rospy.init_node('ir_publish')
    
    spub = SensorPublish()
    rospy.spin()
    
