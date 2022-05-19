#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import socket
import sys, os
import math, numpy
import thread
import traceback

# total number of robots
N = 6
# initial count of color
black_count = 0
white_count = 0
vote_id = 0
print(vote_id)
neighbor_list=[]

threadCurrentlyRunning = False
consensusReached = False
STOPLOOP = False
# initial state: explore
STATE = 1
byzantineStyle = '0'
byzantine_robots = []
##### determine the color of one pixel ########################
def detect_color(array):
    #print("+++++++++++++++++++++++++++++++++++++++++array"+str(array))
    if array[0] < 128 and array[1] < 128 and array[2] < 128: #[0,0,0] is black
        return 1 #black
    else:
        return 0

###### process image array from "/camera" topic ##############
def image_process(img_array):
    # print('-----------image_process-----------')
    black_num = 0

    row = len(img_array)
    colum = len(img_array[0])
    half = row * colum / 2
    for i in range(row):
        for j in range(colum):
            if detect_color(img_array[i][j]) == 1:
                black_num = black_num + 1

        if black_num > half:
            return 1
    return 0

###### callback of Suberscriber, count the color ############
def camera_callback(data):
    global black_count
    global white_count
    try:
        cv_bridge = CvBridge()
        cv_image = cv_bridge.imgmsg_to_cv2(data, "rgb8")
        flag = image_process(cv_image)
        if flag:
            black_count = black_count + 1
        else:
            white_count = white_count + 1

    except CvBridgeError as e:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        return

def neighbor_callback(data):
    global neighbor_list
    neighbors_list = list(data.data)
    try:
        neighbor_list=neighbors_list[N*robot_id:N*(robot_id+1)]
    except Exception as e:
        rospy.logerr('neighbor_callback,' + str(e))
        return

###### subscribe the "/camera" topic       
def camera_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("camera", Image, camera_callback)
    rospy.Rate(10)


def neighbor_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber("/adjmat", Int32MultiArray, neighbor_callback)
    rospy.Rate(10)

####################################################################################################################
####################################################################################################################

def stop():
    try:
        os.system("ROS_NAMESPACE="+str(robot_id)+" rosrun epuck_driver_cpp stop.py")
    except Exception as e:
        pritn(e)


####################################################################################################################

###### get a list of ids of robots in neighborhood ########################
# in this case (for real robots scenario), the communication range is global, so the neighbors are all swarm robots
def getNeighbors():
    return neighbor_list

#vote to blockchain and get response
def Vote(msg):
    try:
        print("vote: "+msg)
        msg=msg.encode()
        s.send(msg)
    except Exception as e:
        print("Error:can't send data to server")
        traceback.print_exc()
        
def Sync(msg):
    try:
        msg=msg.encode()
        s.send(msg)
    except Exception as e:
        #print("Error:can't Sync")
        traceback.print_exc()
        pass

def Explore():
    global black_count
    global white_count
    global start
    global start2
    global start3
    global period
    global STATE
    global consensusReached
    global vote_id

    if remainingExplorationTime > 0:
        # if remainingExplorationTime % 3 == 0:
        #     if consensusReached == True:
        #         msg="#"+str([neighbors,robot_id,"stop"])+'~'
        #     else:
        #         msg="#"+str([neighbors,robot_id,'nan'])+'~'
        #     Sync(msg)
        pass
    else:
        ready_to_vote = True
        if robot_id in byzantine_robots:
            if byzantineStyle == '0':
                quality = 0.0
            elif byzantineStyle == '1':
                quality = 1.0
            else:
                quality = random.uniform(0.0,1.0)
        else:
            if black_count + white_count > 0:
                quality = 1.0 * black_count / (black_count + white_count)
            else:
                quality = 0.0
            if abs(quality - 0.0) < 0.1 or abs(quality - 1.0) < 0.1:
                quality = -1 
        if ready_to_vote:
            neighbors = getNeighbors() 
            if not neighbors:
                neighbors = [0]*N
            if consensusReached == True:
                msg="#[%s, %2d, %6d, %6d]~" % (str(neighbors),robot_id,-2,vote_id)
            else:
                vote_id = vote_id + 1
                msg="#[%s, %2d, %s, %6d]~" % (str(neighbors),robot_id,str(quality).rjust(18,' '),vote_id)
            #send msg to hashgraph
            if time2vote < 0 or time2vote >= 0:
                Vote(msg)    
                start3 = time.clock()
        if remaining2reset <= 0:               
            #重置reset
            black_count = 0
            white_count = 0
            start2 = time.clock()

        start = time.clock()
        #period = math.ceil(numpy.random.exponential(5))
        period = math.ceil(1)
        STATE = 2

def getResult():
    res = ""
    try:
        res = s.recv(1024).decode().split("#")[1].strip("~")
    except Exception as e:
        #traceback.print_exc()
        res = ""
    finally:
        return res

def WaitForDecision(threadName):
    global threadCurrentlyRunning
    global STOPLOOP
    global consensusReached
    threadCurrentlyRunning = False
    result = getResult()
    #print(result)
    if result != "":
        if "end" != result:
            print("C:Response from Server:" + result)
            resultList=eval(result)
            if resultList[0] == True and consensusReached == False:
                print("consensusReached is " + str(resultList[0]))
                #f = open("./data/all_result.txt", "a+")
                #f.write(str(resultList[1:]))
                #f.write('\n')
                #f.close()
                consensusReached = True
        else:
            print("C:end...")
            STOPLOOP = True

def Diffusing():
    global threadCurrentlyRunning
    global STATE
    #change to STATE 1: Explore
    STATE = 1                     
    if not threadCurrentlyRunning:
        threadCurrentlyRunning = True
        try:
            thread.start_new_thread(WaitForDecision,("Thread-1",))           
        except Exception as e:
            traceback.print_exc()

############################# main loop #########################################

if __name__ == '__main__':
    args = sys.argv
    robot_id = eval(args[1])
    #robot_ip = args[2]

    # monitor "/camera" topic
    camera_listener()
    neighbor_listener()
    #connect to hashgraph server
    IP = "172.17.0.1"
    PORT=9955+robot_id
    try:
        socket.setdefaulttimeout(5)
        s = socket.socket()
        s.connect((IP, PORT))   
    except Exception as e:
        print("connection failed")
        traceback.print_exc()    
    else:
        print("connection success")     
    finally: 
        # main loop #
        period = 1
        period4vote = 1
        period4reset = 20
        start = time.clock()
        start2 = time.clock()
        start3 = time.clock()
        while 1:       
            current = time.clock()
            time_diff = int(current - start)
            time_diff2 = int(current -start2)
            time_diff3 = int(current -start3)
            remainingExplorationTime = period - time_diff
            remaining2reset = period4reset - time_diff2
            time2vote = period4vote - time_diff3
            #print(remainingExplorationTime)
            if STATE == 1:  
                Explore()       
            elif STATE == 2:
                Diffusing() 
            if STOPLOOP == True:
                stop()
                break
        s.close()




            

