#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import Int32MultiArray
import time,random
#ps_sensor_name = ['ps0','ps1','ps2','ps3','ps4','ps5','psvelocity','ps7']
ps_sensor_value = ()
velocity = 6
angular_speed = 1
twist = Twist()
twist.linear.x = velocity
rospy.init_node('avoid_obstacle', anonymous = False)
rate = rospy.Rate(100)
pub = rospy.Publisher('mobile_base/cmd_vel_nostop', Twist, queue_size = 1)
last_front_obstacle = 0   
action_time = 0
def randomMove():
    print("randomMove\n")
    random_num = -1+2*random.random()
    twist.linear.x = velocity * random_num
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular_speed * random_num
    pub.publish(twist)

def turnLeft():
    print("turnLeft\n")
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0 
    twist.angular.x = 0
    twist.angular.y = 0 
    twist.angular.z = angular_speed + 1*random.random()
    pub.publish(twist)
    rate.sleep()

def turnRight():
    print("turnRight\n")
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -angular_speed - 1*random.random() 
    pub.publish(twist)
    rate.sleep()

def forward():
    print("forward\n")
    twist.linear.x = velocity
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    rate.sleep()

def backward():
    print("backward\n")
    twist.linear.x = -velocity
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    rate.sleep()

def doAction():
    print("doAction")

def doLastAction():
    print("doLastAction")

def callback(msg):
    global last_front_obstacle
    global action_time
    global doAction
    global doLastAction
    print(msg.data)   # turple
    dis =  100
    ps_sensor_value = msg.data

    front_obstacle=ps_sensor_value[0]>dis or ps_sensor_value[7]>dis
    right_obstacle=ps_sensor_value[0]>dis or ps_sensor_value[1]>dis or ps_sensor_value[2]>dis
    left_obstacle=ps_sensor_value[6]>dis or ps_sensor_value[7]>dis or ps_sensor_value[5]>dis
    back_obstacle=ps_sensor_value[3]>dis or ps_sensor_value[4]>dis

    if front_obstacle:
        doAction = backward

    elif doLastAction == backward:
        # if random.choice([True, False]):
        doAction = turnLeft 
        # else:
        #     doAction = turnRight

    elif back_obstacle:
        doAction = forward
    elif right_obstacle:        
        doAction = turnLeft 
    elif left_obstacle:       
        doAction = turnRight

    else:
        doAction = forward
    # last_front_obstacle = front_obstacle      
    # if doLastAction == turnLeft or doLastAction==turnRight:
    #     if action_time > 1:
    #         doAction = doLastAction
    #         doAction()
    #         action_time = action_time - 1
    #     elif action_time == 1:
    #         doAction = forward
    #     else:
    #         doAction = forward
    #         action_time = 2
    # elif doLastAction == backward or doLastAction == forward or doLastAction == randomMove:
    #     doAction()
    doAction()
    doLastAction = doAction 
'''
    forward()
    if back_obstacle:
        turnRight()
        forward()
        print("forward to continue1\n")
    elif right_obstacle and left_obstacle:
        turnRight()
        backward()
        print("backward to continue2\n")
    elif right_obstacle:
        turnLeft()
        print("turnLeft1\n")
    elif left_obstacle:
        turnRight()
        print("turnRight1\n")
    else:
        forward()
        print("forward to continue2\n")
'''
'''
    if front_obstacle:
        backward()
        time.sleep(1)
        if random.choice([True, False]):
            turnLeft()
        else:
            turnRight()
        time.sleep(1)
    elif right_obstacle:
        turnLeft()
    elif left_obstacle:
        turnRight()
    else:
        forward()
'''
'''
    right_obstacle=ps_sensor_value[0]>dis or ps_sensor_value[1]>dis or ps_sensor_value[2]>dis
    left_obstacle=ps_sensor_value[6]>dis or ps_sensor_value[7]>dis or ps_sensor_value[5]>dis
    back_obstacle=ps_sensor_value[3]>dis and ps_sensor_value[4]>dis
'''




def main():
 
    #pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    rospy.Subscriber("ps0to7",Int32MultiArray,callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

