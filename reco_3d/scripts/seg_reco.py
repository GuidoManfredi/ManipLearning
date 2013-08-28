#!/usr/bin/env python
import roslib; roslib.load_manifest('segment_plans_objects')

import sys
import rospy
from segment_plans_objects.srv import *

segmentation_service_name = '/segmentation'
recognition_service_name = '/recognition_obb'

def segmentation_recognition ():
		clusters = segmentation_client()
		
		for cluster in clusters
			names = recognition_client (cluster);
			result.insert (names[0])

def segmentation_client():
  print "Waiting for segmentation service"
  rospy.wait_for_service(segmentation_service_name)
  print "Calling segmentation..."
  try:
    segment = rospy.ServiceProxy(segmentation_service_name, PlantopSegmentation)
    res = segment()
    return len(res.clusters)
  except rospy.ServiceException, e:
    print "Segmentation service call failed: %s" % e

def recognition_client(cluster):
  print "Waiting for segmentation service"
  rospy.wait_for_service(recognition_service_name)
  print "Calling segmentation..."
  try:
    recognize_obb = rospy.ServiceProxy(recognition_service_name, OrientedBoundingBoxRecognition)
    res = recognize_obb()
    return res.names
  except rospy.ServiceException, e:
    print "Segmentation service call failed: %s" % e

if __name__ == "__main__":
  names = segmentation_recognition()
  print "Found %d clusters" % num_clusters
    
