#!/usr/bin/env python
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool, UInt8
from tello_driver.msg import TelloStatus
from time import sleep
from h264_image_transport.msg import H264Packet
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


global cont
cont = True


def subscribe_video():
  print("subscribe video")
  rospy.init_node('tello_node',anonymous=True) 
  sub = rospy.Subscriber('/tello/image_raw', Image,image_callback) 
  print(sub)
  rospy.spin()

def image_callback(data):
  print("image callback")
  bridge = CvBridge()
  try:
    img = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  cv2.imshow('Original', img)
  cv2.imshow('Canny', cv2.Canny(img, 100, 200))
  cv2.waitKey(1)
  #pic_name = '~/tello.jpeg'
  #with open(pic_name,'wb') as fd:
  #  fd.write(img)
  #pass

def init():
  print("init") 
  rospy.init_node('tello_node',anonymous=True)


def takeOff():
  print("takeoff")
  takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size =1)
  #rospy.init_node('tello_node',anonymous=True)
  sleep(3)
  msg = Empty()
  takeoff_pub.publish(msg)
  print("TakeOff done")

def rotate(axis,value):
  #transfer deg to rad
  value=float(value*3.14159/360)
  step=2
  print("rotate")
  cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size =step)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    for i in range(step):
      print(i)
      msg=Twist()
      setattr(msg.angular,axis,value/step)
      cmd_pub.publish(msg)
      rate.sleep()
    break
  sleep(3)
  print("rotate done")
  
def move(axis,value):
  value=float(value)
  step=10
  print("move")
  cmd_pub =  rospy.Publisher('/tello/cmd_vel', Twist, queue_size =step)
  rate = rospy.Rate(step)
  
  while not rospy.is_shutdown():
    for i in range(step):
      print(i)
      msg=Twist()
      setattr(msg.linear,axis,value/step)
      cmd_pub.publish(msg)
      rate.sleep()
    break
  sleep(5)
  print("move done")


def fastmode():
  print("fast mode")
  pub =  rospy.Publisher('/tello/fast_mode', Bool, queue_size =1)
  pub.publish(Bool(True))
  sleep(1)

def land():
  print("Land")
  global cont
  status_sub = rospy.Subscriber('/tello/status', TelloStatus,callback)
  land_pub = rospy.Publisher('/tello/land', Empty, queue_size =1)
  rate = rospy.Rate(10)
  try_count =0  
  while (cont==True and try_count <20):
    print("try land")
    try_count +=1
    msg=Empty()
    land_pub.publish(msg)
    rate.sleep()

def callback(data):
  global cont
  print("callback")
  cont=False   
  if data:
     cont = data.is_flying
     print("read data")

def flip():
  print("flip")
  rospy.init_node('tello_node',anonymous=True)
  flip_pub = rospy.Publisher('/tello/flip', UInt8 , queue_size =1)
  msg = UInt8(5)
  print(msg.data)
  msg.data = 1
  flip_pub.publish(msg)
  sleep(3)  

def action1():
  try:
    init()
    fastmode()
    takeOff()
    move("z",1)
    move("x",1)
    rotate("z",90)
    move("y",1)
    land()
  except rospy.ROSInterruptException:
    print("failed")
    pass


def action2():
  try:
    init()
    subscribe_video()
    sleep(3)

  except rospy.ROSInterruptException:
    print("failed")
    pass

def action3():
  try:
    init()
    fastmode()    
    takeOff()
    #sleep(2)
    flip()
    land()
  except rospy.ROSInterruptException:
    print("failed")
    pass


if __name__ =='__main__':
  action2()
