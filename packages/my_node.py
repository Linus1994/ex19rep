#!/usr/bin/env python 
import rosbag
import os
import numpy as np

topicList = []

ros_bag = os.environ['ROSBAG']
bag = rosbag.Bag(ros_bag)

for topic, msg, t in bag.read_messages():
        if topicList.count(topic) == 0:
            topicList.append (topic)


for topic in topicList:
    timestamp = []
    timediff  = []
    
    for topic, msg, t in bag.read_messages(topics=topic):
        timestamp.append(msg.header.stamp)
    for j in range(len(timestamp)-1):
        timediff.append(timestamp[j+1]-timestamp[j])

    timemin = np.min(timediff)
    timemax = np.max(timediff)
    timemean = np.mean(timediff)
    timemedian = np.median(timediff)

    print topic,":"
    print(" num_messages: %i"%len(timestamp))
    print(" Period:")
    print "     min:", timemin, "ns"
    print "     max:", timemax, "ns"
    print "     average:", timemean, "ns"
    print "     median:", timemedian, "ns\n"







# camtimestamps = []
# camdiff = []
# for topic, msg, t in bag.read_messages(topics=['/tesla/camera_node/camera_info']):#, 'numbers']):
     # camtimestamps.append(msg.header.stamp)
  
# #camtimear = np.asarray(camtimestamps)
# for j in range(len(camtimestamps)-1):
    # camdiff.append(camtimestamps[j+1]-camtimestamps[j])

# cammin = np.min(camdiff)
# cammax = np.max(camdiff)
# cammean = np.mean(camdiff)
# cammedian = np.median(camdiff)

# print(topic)
# print(" num_messages: %i"%len(camtimestamps))
# print(" Period:")
# print "     min:", cammin, "ns"
# print "     max:", cammax, "ns"
# print "     average:", cammean, "ns"
# print "     median:", cammedian, "ns\n"


# segstamps = []
# segdiff = []
# bag = rosbag.Bag('/home/example_rosbag_H3.bag')
# #/linuslingg/wheels_driver_node/wheels_cmd
# for topic, msg, t in bag.read_messages(topics=['/tesla/line_detector_node/segment_list']):#, 'numbers']):#, 'numbers']):
     # segstamps.append(msg.header.stamp)
  
# #camtimear = np.asarray(camtimestamps)
# for j in range(len(segstamps)-1):
    # segdiff.append(segstamps[j+1]-segstamps[j])

# segmin = np.min(segdiff)
# segmax = np.max(segdiff)
# segmean = np.mean(segdiff)
# segmedian = np.median(segdiff)

# print(topic)
# print(" num_messages: %i"%len(segstamps))
# print(" Period:")
# print "     min:", segmin, "ns"
# print "     max:", segmax, "ns"
# print "     average:", segmean, "ns"
# print "     median:", segmedian, "ns\n"

# wheelstamps = []
# wheeldiff = []
# bag = rosbag.Bag('/home/example_rosbag_H3.bag')
# for topic, msg, t in bag.read_messages(topics=['/tesla/wheels_driver_node/wheels_cmd']):#, 'numbers']):#, 'numbers']):
     # wheelstamps.append(msg.header.stamp)
  
# #camtimear = np.asarray(camtimestamps)
# for j in range(len(wheelstamps)-1):
    # wheeldiff.append(wheelstamps[j+1]-wheelstamps[j])

# wheelgmin = np.min(wheeldiff)
# wheelmax = np.max(wheeldiff)
# wheelmean = np.mean(wheeldiff)
# wheelmedian = np.median(wheeldiff)

# print(topic)
# print(" num_messages: %i"%len(wheelstamps))
# print(" Period:")
# print "     min:", wheelgmin, "ns"
# print "     max:", wheelmax, "ns"
# print "     average:", wheelmean, "ns"
# print "     median:", wheelmedian, "ns"


#dts devel build -f --arch amd64
#docker run -it --rm -v ~/bags2:/home duckietown/ex19rep:v1-amd64
