#!/usr/bin/env python
# coding: utf-8 

import cv2
import rospy
from nav_msgs.msg import OccupancyGrid

import time
import numpy as np
import matplotlib.pyplot as plt

class Map(object):
  def __init__(self):
    self.map_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.callback)
    print self.map_sub

  def callback(self, mapmsg):
    try:
      map = mapmsg.data
      print type(map)
      map = np.array(map)
      print map.shape
      map = map.reshape((80,80))
      print map
      row,col = map.shape
      print row,col
      tem = np.zeros((row,col))
      for i in range(row):
        for j in range(col):
          if(map[i,j]==-1):
             tem[i,j]=255
          else:
             tem[i,j]=map[i,j]
      print map.shape
      cv2.imshow("map",tem)
      cv2.waitKey(30)
      # plt.imshow(map)
      # plt.draw()
      # plt.show()
      # plt.pause(0.01)
    except Exception,e:
      print e
      rospy.loginfo('convert rgb image error')

  def getImage():
    return self.rgb_image

def main(_):
  rospy.init_node('map', anonymous=True)
  v=Map()
  rospy.spin()

if __name__=='__main__':
  main('_')

