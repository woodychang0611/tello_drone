#!/usr/bin/env python
import rospy
import roslib
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
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
  sub = rospy.Subscriber('/tello/image_raw', Image,image_callback) 
  print(sub)

def image_callback(data):
  print("image callback")
  bridge = CvBridge()
  try:
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    print('take over')
  except CvBridgeError as e:
    print(e)
  cv2.imshow('Original', img)
  cv2.imshow('Canny', cv2.Canny(img, 100, 200))
  cv2.waitkey(1)
  pic_name = '~/tello.jpeg'
  with open(pic_name,'wb') as fd:
    fd.write(img)
  pass

def TakeOff():
  print("Takeoff")
  takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size =1)
  rospy.init_node('tello_node',anonymous=True)
  sleep(1)
  msg = Empty()
  takeoff_pub.publish(msg)
  print("TakeOff done")

def rotate(axis,value):
  #transfer deg to rad
  value=float(value*3.14159/180)
  step=5
  print("rotate")
  cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size =step)
  rate = rospy.Rate(step/2.0)

  while not rospy.is_shutdown():
    for i in range(step):
      print(i)
      msg=Twist()
      setattr(msg.angular,axis,value/step)
      cmd_pub.publish(msg)
      rate.sleep()
    break
  sleep(5)
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


def Land():
  print("Land")
  global cont
  status_sub = rospy.Subscriber('/tello/status', TelloStatus,callback)
  land_pub = rospy.Publisher('/tello/land', Empty, queue_size =1)
  rate = rospy.Rate(10)
  try_count =0  
  while (cont ==True and try_count <20):
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

if __name__ =='__main__':
  try:
    subscribe_video()
    sleep(1)
    TakeOff()
    sleep(1)
    move("z",2)
    sleep(1)
    move("x",1)
    sleep(1)
    rotate("z",360)
    sleep(1)
    move("x",1)
    sleep(1)    
    Land()
  except rospy.ROSInterruptException:
    print("failed")
    pass    
  finally:
    sys.exit(0)


