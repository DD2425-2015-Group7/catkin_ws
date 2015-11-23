#!/usr/bin/env python

import random
import csv
import rospy
import rosbag
from detection.msg import BoundingBox
from detection.msg import BoundingBoxArray
from sensor_msgs.msg import CompressedImage

def loadBBs(bagFile, topicName):
    bag = rosbag.Bag(bagFile)
    bbs = []
    for topic, msg, t in bag.read_messages(topics=[topicName]):
        if len(msg.bounding_boxes) <=0:
            continue;
        bbs += [msg]
    bag.close()
    return bbs

def sampleBBs(bbs, maxImages, maxPerImage):
    sb = []
    pool = []
    assert(len(bbs)>=maxImages)
    for i in xrange(0, len(bbs)):
        pool += [i]
        # Favour images with more bounding boxes.
        if len(bbs[i].bounding_boxes)>=20:
            pool += [i]
    lst = random.sample(pool, maxImages)
    # But do not leave duplicates.
    lst = list(set(lst))
    lst.sort()

    n = 1
    for i in lst:
        imbb = bbs[i]
        bbCount = len(imbb.bounding_boxes)
        assert(bbCount>0)
        if bbCount > maxPerImage:
            smp = random.sample(xrange(0, bbCount), maxPerImage)
        else:
            smp = xrange(0, bbCount)
        for j in smp:
            bb = []
            bb += [imbb.header.stamp.secs]
            bb += [imbb.header.stamp.nsecs]
            bb += [n]
            bb += [imbb.bounding_boxes[j].x0]
            bb += [imbb.bounding_boxes[j].y0]
            bb += [imbb.bounding_boxes[j].x1]
            bb += [imbb.bounding_boxes[j].y1]
            sb += [bb]
        n += 1
    return sb

def extractImages(imgBagFile, topicName, bbs, pub):
    rate = rospy.Rate(4)
    bag = rosbag.Bag(imgBagFile)
    i = 0
    curSecs = bbs[i][0]
    curNsecs = bbs[i][1]
    n = 0
    for topic, msg, t in bag.read_messages(topics=[topicName]):
        if msg.header.stamp.secs > curSecs:
            rospy.logerr("sample_bag ERROR: message with stamp.secs {} missing in the bag!".format(curSecs))
            break
        if msg.header.stamp.secs != curSecs:
            continue
        if msg.header.stamp.nsecs != curNsecs:
            continue
        pub.publish(msg)
        n += 1
        rate.sleep()
        while i < len(bbs):
            if bbs[i][0] == curSecs and bbs[i][1] == curNsecs:
                i += 1
            else:
                break
        if i>=len(bbs):
            break
        else:
            assert(curSecs < bbs[i][0] or curNsecs < bbs[i][1])
            curSecs = bbs[i][0]
            curNsecs = bbs[i][1]
    bag.close()
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.loginfo("sample_bag sent {} images for extraction\n".format(n))
    if(i < len(bbs)):
        rospy.logerr("sample_bag ERROR: not enough images in the bag!")
        return False
    else:
        return True



def saveAnnotation(bbs, fileName, sessionID):
    with open(fileName, 'wb') as csvfile:
        awriter = csv.writer(csvfile, delimiter='\t')
        for bb in bbs:
            awriter.writerow(["negative_boost/data/neg{}-{}.png".format(sessionID, bb[2]), -1, bb[3], bb[4], bb[5], bb[6]])


if __name__ == '__main__':
    rospy.init_node('sample_bag')
    sessionID = rospy.get_param('~boost_session_id', 1)
    imgBagFile = rospy.get_param('~img_bag', "")
    bbBagFile = rospy.get_param('~bb_bag', "")
    csvFile = rospy.get_param('~sample_csv_file', "")
    rate = rospy.Rate(10)
    img_pub = rospy.Publisher("/republish/image_rect_color_compressed/compressed", CompressedImage, queue_size=2)
    rospy.loginfo("sample_bag running")

    random.seed()
    bbs = loadBBs(bbBagFile, "/object_detector/bounding_boxes")
    bbs = sampleBBs(bbs, 60, 10)
    if(extractImages(imgBagFile, "/camera/rgb/image_rect_color_compressed/compressed", bbs, img_pub)):
        rospy.loginfo("sample_bag extracted {} bounding boxes".format(len(bbs)))
        saveAnnotation(bbs, csvFile, sessionID)

