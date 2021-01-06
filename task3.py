#!/usr/bin/env python
import rospy
import roslib
import tello_drone
import sys
import argparse
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from behave import *
from time import sleep

random.seed()
parser = argparse.ArgumentParser()
parser.add_argument('-sim', action='store_true') 
parser.parse_args()
args = parser.parse_args()
print(args.sim)
sim_mode = args.sim
if (sim_mode):
    print("sim mode")
else:
    rospy.init_node('bt_mission', anonymous=True)
    sleep(2)
    print("takeoff")
    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size =1)
    sleep(3)
    msg = Empty()
    takeoff_pub.publish(msg)
    print("TakeOff done")
    sleep(3)


class bt_mission:
    global sim_mode
    # common member
    isContinue = True
    color ="red"
    if (not sim_mode):
      drone = tello_drone.Drone()
      center = (480, 180)
      cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 10)
      land_pub = rospy.Publisher('/tello/land', Empty, queue_size = 1)
      change_pub = rospy.Publisher('/selfChanged', Empty, queue_size = 10)
      rate = rospy.Rate(10)

    def __init__(self):
        self.tree = (
            self.RedNotFinish >> (self.NotReady2Pass | self.PassAndSwitch) >> ( (self.isNotCenter >> self.FixedPose) | (self.isCenter >> self.Forward) ) >> (self.rec_over1 | self.hover)
            |self.BlueNotFinish >> (self.NotReady2Pass | self.PassAndLand) >> ( (self.isNotCenter >> self.FixedPose) | (self.isCenter >> self.Forward) ) >> (self.rec_over1 | self.hover)
        )

    @condition
    def BlueNotFinish(self):
        print("condition: BlueNotFinish")
        print(bt_mission.color == "blue")
        return bt_mission.color == "blue"

    @condition
    def RedNotFinish(self):
        print("condition: RedNotFinish")
        print(bt_mission.color == "red")
        return bt_mission.color == "red"

    @condition
    def NotReady2Pass(self):
        print("condition: NotReady2Pass")
        if (sim_mode):
           ret =  random.random()>0.9
           print(ret)
           return ret
        return bt_mission.drone.suber.target[2] != -1

    @condition
    def isNotCenter(self):
        print("condition: isNotCenter")
        if (sim_mode):
           ret = random.random()>0.9
           print(ret)
           return ret
        return abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) > 60 or abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) > 30

    @condition
    def isCenter(self):
        print("condition: isCenter")
        if (sim_mode):
           ret = random.random()>0.3
           print(ret)        
           return ret
        return abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) <= 60 and abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) <= 30

    @condition
    def rec_over1(self):
        if (sim_mode):
           ret = False
           print(ret)
           return ret
        print("condition: rec_over1")
        return rospy.get_time() - bt_mission.drone.suber.rec_time <= 1.0
    @action
    def PassAndSwitch(self):
        print("action: PassAndSwitch")
        if (sim_mode):
            sleep(0.5)
            print("=================rotate===================")  
            bt_mission.color="blue"
            return 
        msg = Twist()
        msg.linear.x = 0.35
        #msg.linear.y = -0.1
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
        msg = Twist()
        msg.linear.x = 0.26 
        msg.linear.z = 0.05
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
        msg = Twist()
        msg.angular.z = 1
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
        bt_mission.change_pub.publish(Empty())
        bt_mission.rate.sleep()
        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        bt_mission.color = "blue"


    @action
    def PassAndLand(self):
        print("action: PassAndLand")
        bt_mission.isContinue=False
        if (sim_mode):
           sleep(1)
           #exit(0) 
           return
        msg = Twist()
        msg.linear.x = 0.24
        #msg.linear.z = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
        msg = Twist()
        msg.linear.x = 0.26 
        #msg.linear.z = 0.2
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()
        sleep(2)
        msg = Twist()
        bt_mission.cmd_pub.publish(msg)
        bt_mission.rate.sleep()

        while bt_mission.drone.suber.canLand is not True:
          msg = Empty()
          bt_mission.land_pub.publish(msg)
          bt_mission.rate.sleep()

    @action
    def FixedPose(self):
      print("action: FixedPose")
      if (sim_mode):
           sleep(0.5)
           return
      msg = Twist()
      if abs(bt_mission.drone.suber.target[0] - bt_mission.center[0]) >= 60:
        msg.linear.y = -(bt_mission.drone.suber.target[0] - bt_mission.center[0]) / abs((bt_mission.drone.suber.target[0] - bt_mission.center[0])) * 0.1
      if abs(bt_mission.drone.suber.target[1] - bt_mission.center[1]) >= 30:
        msg.linear.z = -(bt_mission.drone.suber.target[1] - bt_mission.center[1]) / abs((bt_mission.drone.suber.target[1] - bt_mission.center[1])) * 0.2
      bt_mission.cmd_pub.publish(msg)
      bt_mission.rate.sleep()

    @action
    def Forward(self):
      print("action: Forward")
      if (sim_mode):
           sleep(0.5)
           return
      msg = Twist()
      msg.linear.x = 0.2
      bt_mission.cmd_pub.publish(msg)
      bt_mission.rate.sleep()

    @action
    def hover(self):
      print("action: hover")
      if (sim_mode):
           sleep(0.5)
           return
      msg = Twist()
      bt_mission.cmd_pub.publish(msg)
      bt_mission.rate.sleep()

    def run(self):
        while True:
            if bt_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print "state = %s\n" % state
            #if bt_mission.drone.isStop == True:
            #  exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print "state = %s\n" % state
                #if bt_mission.drone.isStop == True:
                #  exec("f = open(\"123.txt\",\'rb\')")
            assert state == SUCCESS or state == FAILURE

def main():
    print("start...") 
    btCm_n = bt_mission()
    sleep(2)

    try:
        btCm_n.run()
        if(sim_mode):
           print("end for sim")
           exit(0)
        rospy.spin()
    except IOError, KeyboardInterrupt:
        print("Shutting down ROS module")

if __name__ == "__main__":
    main()
