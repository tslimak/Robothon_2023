#!/usr/bin/python3

import os
import cv2
import numpy as np

from __future__ import print_function
from vision.srv import TriangleDetect,TriangleDetectResponse
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError


#################################################################################################
## functions for triangle detection
#################################################################################################

# colorThresholding -> preperation for finding contours
# input:
#   img = input image
#   col_space = color space which should be used as a basis for thresholding
#       col_space = 0 -> BGR
#       col_space = 1 -> Lab
#   channel = channel of the color space
#       channel = 3 -> e.g. BGR color space -> only look an R
#   thr_perc = threshold value in percent
#   erode = size of erode kernel
# output:
#   thr_Img = binary image (black & white)
def col_thr(img, col_space=0, channel=3, thr_perc=0.7, erode=0):
  col_img = img.copy()

  # convert image to chosen color space
  # 0 = nonConvert; 1 = Lab
  if col_space == 1:
    col_img = cv2.cvtColor(col_img, cv2.COLOR_BGR2LAB)
  
  # get single channel from image
  ch = col_img[:,:,channel]

  # percentual thresholding for single color channel
  thr, thr_img = cv2.threshold(ch, (thr_perc*np.max(ch)), np.max(ch), cv2.THRESH_BINARY)
  #print('threshold: ' + str(thr))

  # erode everything -> if triangles are aligned they get more separated for detection
  if erode > 0:
    # kernel size
    size = 3 #3 works best!
    kernel = np.ones((size, size), np.uint8)

    thr_img = cv2.erode(thr_img, kernel, iterations=1)

  return thr_img


# shape_detect -> finds specific shapes
# -> depending on the number of vertex points different shapes can be detected
#     vert_p = 3 -> triangle
#     vert_p = 4 -> rectangle
def shape_detect(img, thr_img, vert_p = 3, perc=[0.15], min_area=30, max_area=100):

  # find contours
  contours,hierarchy = cv2.findContours(thr_img, cv2.RETR_TREE,  cv2.CHAIN_APPROX_NONE)
  #print(len(contours))

  print('Check for shape')
  #print(len(contours))

  shape_cont = []

  for p in perc:
    #print(' ')
    #print(p)
    #print('------------------')
    for cnt in range(len(contours)):
      #check_img = img.copy()

      area = cv2.contourArea(contours[cnt])
      #print('contourArea: ' + str(area))

      #all_cont_img = cv2.drawContours(check_img, contours[cnt], -1, (0,0,255), 1)
      #cv2_imshow(all_cont_img)

      approx = cv2.approxPolyDP(contours[cnt], p*cv2.arcLength(contours[cnt], True), True)
      #print(len(approx))

      # if a contour has 3 vertex points = triangle
      if len(approx) == vert_p and area > min_area and area < max_area:
        print('Found shape!')

        #all_cont_img = cv2.drawContours(check_img, contours[cnt], -1, (0,255,0), 2)
        #cv2_imshow(all_cont_img)

        shape_cont.append(cnt)
            
        #print('')

  #print('------------------')
  shape_cont = np.take(contours, shape_cont)
  #print(len(shape_cont))
  #print(shape_cont.shape)

  return shape_cont


# crop_img -> crops the original image to the point of view
# -> reduces distractive noise
def crop_img(img, shape_cont):
  #print(len(shape_cont))
  if len(shape_cont) == 1:
    # how does the contour looks like
    sh = shape_cont[0]
    #print(sh.shape)
    #print(sh)
    min_vertex = np.amin(sh, axis=0)
    max_vertex = np.amax(sh, axis=0)
    #print(min_vertex)
    #print(max_vertex)
 
    #cp_img = img.copy()

    cp_img = cp_img[min_vertex[0][1]:max_vertex[0][1], min_vertex[0][0]:max_vertex[0][0]]
    #cv2_imshow(cp_img)

  else:
    print('No Display was found!')

  return cp_img


# calc_dist -> calculate distance between triangle vertex points
# -> determines the distance for slider in meter
def calc_dist(crop_img, tri_cont):

  # get top & bottom vertex of each triangle
  top_vert = []
  bot_vert = []

  if len(tri_cont) == 2 or len(tri_cont) == 3:
    for tri in tri_cont:
      #vert_img = crop_img.copy()
      #cv2_imshow(cv2.drawContours(vert_img, tri, -1, (0,0,255), 1))

      top_vert.append(tuple(tri[tri[:,:,0].argmin()][0]))
      bot_vert.append(tuple(tri[tri[:,:,0].argmax()][0]))
  else:
    print("There are multiple triangles!")

  #print(len(top_vert))
  #print(len(bot_vert))

  # calculate distance between vertex points
  top_dist = np.subtract(np.array(top_vert[0]), np.array(top_vert[1]))
  print('top vertex distance: ' + str(top_dist))

  bot_dist = np.subtract(np.array(bot_vert[0]), np.array(bot_vert[1]))
  print('bottom vertex distance: ' + str(bot_dist))


  # calculate average distance
  avg_dist = np.mean(np.stack((top_dist,bot_dist)), axis=0)
  print('average distances: ' + str(avg_dist))

  # Distance need to be driven
  #print(' ')
  dist = avg_dist[0]
  print('gripper need to drive linear distance of ' + str(dist) + ' pixel')

  # convert pixel to display
  dist = (0.2/105) * dist
  print('pixel to display convertion: ' + str(dist))
  # convert display to slider
  dist = (0.3/0.2) * dist
  print('gripper need to drive linear distance of ' + str(dist) + ' meter')

  return dist

#################################################################################################
## function for ROS nodes, server definitions
#################################################################################################

""" def handle_triangle_detection(req):
  print('Triangle Detection: ' + str(req+1))
  # create a ROS node with the name 'imageGrapper' -> get's image
  # anonymous=True -> adds unique number to the node name
  rospy.init_node('imageGrapper', anonymous=True)
  # node subscribes to topic '/camera/color/image_raw' with type Image
  # when a message is received the image_callback function is processing the Image
  rospy.Subscriber('/camera/color/image_raw', Image, image_callback, (req)) 
  
  
  def image_callback(sensor_image, args):
  
  """


# todo: Wie kann ich der Funktion 'image_callback' ebenfalls req übergeben?
# -> answer.ros.org/question/332126/passing-multiple-arguments-to-subscriber-callback-function-in-python/
# -> answers.ros.org/question/231492/passing-arguments-to-callback-in-python/

# sensor_msgs/Image Message
# docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html


def handle_triangle_detection(req):

  print('received request parameter: ' + str(req.triangleID))


  # get only one message from topic
  sensor_image = rospy.wait_for_message('/camera/color/image_raw', Image)
  image = np.frombuffer(sensor_image.data, dtype=np.unit8).reshape(sensor_image.heigth, sensor_image.width, -1)
  cv2.imshow("Camera Image", image)

  # processing the image from real sense
  # req.triangleID -> will be used to decide which picture it is (frist -> 0 or second -> 1)
  
  ## get display
  # get only read parts
  thr_img = col_Thr(image, col_space=1, channel=1, thr_perc=0.78, erode=0)
  # find rectangle = display
  shape_cont = shape_detect(image, thr_img, vert_p=4, perc=[0.1], min_area=5000, max_area=7000)
  # crop image -> image = only display
  cp_img = crop_img(image, shape_cont)

  ## detect triangle
  thr_img = col_thr(cp_img, col_space=0, channel=2, thr_perc=0.68, erode=0) # erode=3)
  # find triangle
  shape_cont = shape_detect(cp_img, thr_img, vert_p=3, perc=[0.2], min_area=10, max_area=100)

  if req.triangleID == 0:
    old_shape_cont = shape_cont
  elif req.triangleID == 1:
    shape_cont = [np.array(shape_cont[0]), np.array(old_shape_cont[0])]

  ## calculate distance slider need to drive
  dist = calc_dist(cp_img, shape_cont)

  # todo: Ist das okay, dass die Callback vom Subscriber die Responsemessage des Servers sendet?
  return TriangleDetectResponse(dist)

def triangle_detection_server():
  rospy.init_node('triangle_detection_server')
  s = rospy.Service('triangle_detection', TriangleDetect, handle_triangle_detection)
  print("Ready to get triggered for image processing!")
  rospy.spin()

if __name__ == "__main__":
  triangle_detection_server()

