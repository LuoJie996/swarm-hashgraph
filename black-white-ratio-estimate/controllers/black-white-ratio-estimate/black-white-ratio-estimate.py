#!/usr/bin/python3
# coding=UTF-8
from controller import Robot,Camera,LED,Supervisor
import time,random,math,numpy
import socket
import _thread as thread
import traceback

robot = Supervisor()
timestep =640 #ms
timeFactor = timestep / 100.0
customData = int(robot.getCustomData())
TURN=45 / timeFactor
LAMDA=100 / timeFactor
sigma=100 / timeFactor
alpha=1.0
sensor_num = 8
ps_sensor_name = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
        
N = 20
group_number = 1
groups = {0:[],1:[],2:[],3:[]}

for i in range(N):
    groups[i % group_number].append(i)
        
f = open("/home/luo/commRanger.txt","r")
X = int(f.readline())
f.close()         
X = X
ranger_robots=list(range(N)[:X])
print("ranger_robots=%s"%str(ranger_robots))
byzantine_number = 0
f = open("/home/luo/ByzNum.txt","r")
byzantine_number = int(f.readline())
f.close() 
byzantine_number = 0
byzantine_robots = random.sample(list(range(N)), byzantine_number)

center = [[0.6,0.6],[-0.6,0.6],[-0.6,-0.6],[0.6,-0.6]]
# X is the amount of ranger_robots in each group

termination_time_ticks = 4000

for i in range(group_number):
    if customData in groups[i]:
        group_id = i


##################################
####       Turn LEDs On       ####         
################################## 
def TurnLeds(): 
    global opinion    
    if opinion == 1:   
        led[1].set(0x00ff00) #green for white
        pass
    elif opinion == 2:  
        led[1].set(0x0000ff) #blue for black

##################################
####        Movement          ####         
################################## 
def Move():
    if direction == 0:
        forward()
    elif direction == 1:
        # turnRight()
        forward()
    elif direction == 2:
        # turnLeft()
         forward()
def randomMove():
    random_num = -1+2*random.random()
    left_motor.setVelocity(random_num*speed_max)
    right_motor.setVelocity(-random_num*speed_max)

def turnLeft():
    left_motor.setVelocity(-speed_max)
    right_motor.setVelocity(speed_max)

def turnRight():
    left_motor.setVelocity(speed_max)
    right_motor.setVelocity(-speed_max)

def forward():
    left_motor.setVelocity(speed_max)
    right_motor.setVelocity(speed_max)

def backward():
    left_motor.setVelocity(-speed_max)
    right_motor.setVelocity(-speed_max)
    
def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
def doAction():
    print("doAction")
def doLastAction():
    print("doLastAction")
##################################
####    obstacle_avoidance     ###         
##################################  
def moveBack():
    stop()
    epuck = robot.getFromDef("epuck"+str(customData))
    self_pos = epuck.getPosition()
    self_rot = epuck.getField("rotation").getSFRotation()
    delta_x = center[group_id][0] - self_pos[0]
    delta_z = center[group_id][1] - self_pos[2]
    theta = math.atan(abs(delta_x / delta_z))
    if delta_x >= 0 and delta_z >= 0:
        theta =  theta-math.pi
    elif delta_x >=0 and delta_z < 0:
        theta =  -theta
    elif delta_x < 0 and delta_z < 0:
        theta = theta
    else:
        theta = -theta-math.pi
    
    epuck.getField("rotation").setSFRotation([self_rot[0],self_rot[1],self_rot[2],theta])
    
def ObstacleAvoidance():
    global last_front_obstacle
    global last_right_obstacle
    global last_left_obstacle
    global action_time
    global doAction
    global doLastAction
    global doMoveBack
    image_array = camera.getImageArray()
    in_river = image_process(image_array,"blue")
    if customData in ranger_robots:
        in_river = 0
    ps_sensor_value = []
    for i_ in range(sensor_num):
        ps_sensor_value.append(ps_sensor[i_].getValue())
    dis =100
    front_obstacle=ps_sensor_value[0]>dis or ps_sensor_value[7]>dis or in_river
    right_obstacle=ps_sensor_value[0]>dis or ps_sensor_value[1]>dis or ps_sensor_value[2]>dis
    left_obstacle=ps_sensor_value[6]>dis or ps_sensor_value[7]>dis or ps_sensor_value[5]>dis
    back_obstacle=ps_sensor_value[3]>dis or ps_sensor_value[4]>dis 
    epuck = robot.getFromDef("epuck"+str(customData))
    self_pos = epuck.getPosition()
    escaped = 0
    arrived = 1
    doRandomMove = 0

              
    if customData in ranger_robots:
        escaped = 0
        arrived = 1
    
    if escaped:
        print("id=%2d escaped" % customData)
        print(self_pos)
        if not doMoveBack:
            print("id=%2d doMoveBack" % customData)
            moveBack()
            doMoveBack = 1
    if not arrived and doMoveBack:
        forward()
    else:
        if front_obstacle:
            doAction = backward
        elif right_obstacle:        
            doAction = turnLeft 
        elif left_obstacle:       
            doAction = turnRight   
        elif last_front_obstacle:
            if random.choice([True, False]):
                doAction = turnLeft 
            else:
                doAction = turnRight
        elif doRandomMove:
            doAction = randomMove
        else:
            doAction = forward
        last_front_obstacle = front_obstacle      
        if doLastAction == turnLeft or doLastAction==turnRight:
            if action_time == 1:
                action_time = action_time - 1
                doAction = randomMove
            if action_time > 0:
                doAction = doLastAction
                doAction()
                action_time = action_time - 1
            else:
                action_time = 3
    
        elif doLastAction == backward or doLastAction == forward or doLastAction == randomMove:
            doAction()
        doLastAction = doAction        
##################################
####        RandomWalk        ####         
##################################
def RandomWalk():
    global remainingTime
    global direction
    ObstacleAvoidance()
    if remainingTime == 0:
        if direction == 0:
            p=random.uniform(0,1)
            p=p*TURN
            dir = random.uniform(-1,1)
            if dir > 0:
                direction = 1
            else:
                direction = 2
            remainingTime = math.floor(p)
        else:
            remainingTime = math.ceil(numpy.random.exponential(LAMDA))
            #print("remainingTime:"+str(remainingTime))
            direction = 0
    else:
        remainingTime = remainingTime - 1

##################################
####        ImageProcess      ####         
##################################     


def detect_color(array):
    if array[0] < 10 and array[1] < 20 and array[2] < 20:
        return "black"
    elif array[0] > 200 and array[1] > 200 and array[2] >200:
        return "white"
    elif abs(array[0]-197) < 10 and abs(array[1]-60) <20 and abs(array[2]-60)<20:
        return "red"
    elif abs(array[0]-0) < 20 and abs(array[1]-207) <20 and abs(array[2]-207)<20:
        return "blue"
    else:
        return "undefined"

def image_process(img_array,color):
    black_num = 0
    row = len(img_array)
    colum = len(img_array[0])
    half = row * colum / 2
    for i in range(row):
        for j in range(colum):
            if detect_color(img_array[i][j]) == color:
                black_num = black_num + 1
        if black_num > half:
            return 1
    return 0
    
def DetectCell():
    global black_count
    global white_count
    global object_pos_list
    object_pos = ()
    image_array = camera.getImageArray()
    flag = image_process(image_array,"black")
    if flag:
        black_count = black_count + 1
    else:
        white_count = white_count + 1   

def callback(data):
    global current_color
    try:
        cv_bridge = CvBridge()
        cv_image = cv_bridge.imgmsg_to_cv2(data, "rgb8")
        resp = image_process(cv_image)
        if resp:
          print("black")
          current_color = 1
        else:
          print("white")
          current_color = 0
    except CvBridgeError as e:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        return
        
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera", Image, callback)
      
##################################
####        Explore           ####         
##################################                        
def Explore():
    global opinion
    global quality
    global black_count
    global white_count
    global remainingExplorationTime
    global STATE
    global vote
    global voteFlag
    
    DetectCell()

    if black_count < white_count:
        opinion = 1    
    else:
        opinion = 2
    quality = 1.0 * alpha * black_count / (black_count + white_count)
    vote = format(quality, '.6f')
    if(remainingExplorationTime > 0):
        remainingExplorationTime = remainingExplorationTime - 1
    else:
        voteFlag = True
                
        remainingExplorationTime = math.ceil(sigma + 100 / timeFactor)
        exploreDurationTime = remainingExplorationTime
        #重置reset
        black_count = 0
        white_count = 0
        STATE = 2                #更改状态为2,Diffusing       
        
def Vote():
    global black_count
    global white_count
    global quality
    global neighbors
    global vote
    global vote_id
    vote_id = vote_id + 1
    msg="#[%s, %2d, %s, %6d]~" % (str(neighbors),customData,str(vote).rjust(18,' '),vote_id)
    print("vote: %s lenth=%d" % (msg,len(msg)))
    msg=msg.encode()
    votes.append(vote)

    print("id==%d, n=%d, votes=%s" % (customData,len(votes),str(votes)))
    try:
        s.send(msg)
    except Exception as e:
        msg=""
        # print("Error:can't send data to server")
        # print(e)
        
def Sync():
    msg = "#[%s, %2d, %s, %6d]~" % (str(neighbors),customData,str(vote).rjust(18,' '),vote_id)
    try:
        # print("sync: "+msg)
        msg=msg.encode()
        s.send(msg)
    except Exception as e:
        msg =""
        # print("Error:can't Sync")
        # print(e)         
        
def Sync_stop_signal():
    try:
        msg="#[%s, %2d, %6d, %6d]~" % (str(neighbors),customData,-2,vote_id)
        print("sync: "+msg)
        msg=msg.encode()
        s.send(msg)
    except Exception as e:
        msg =""
        # print("Error:can't Sync")
        # print(e)
 
##################################
####        Diffusing         ####         
##################################   
def ConnectAndListen():
    global neighbors
    global consensusReached
    global remainingsyncTime
    global voteFlag
    global STOPLOOP
    remainingsyncTime=remainingsyncTime-1
        
    neighbors=getNeighbors()
            
    diffneighbors = updateNeighbors()

    if voteFlag and not consensusReached and termination_time_ticks > 0:
        Vote()
        voteFlag = False
    else:
        # remainingsyncTime = math.ceil(20 / timeFactor)
        remainingsyncTime=-1
        if consensusReached:
            Sync_stop_signal()
        else:
            Sync()


def getDistance(pos1,pos2):
    dist=(pos1[0]-pos2[0])**2+(pos1[2]-pos2[2])**2
    return dist
        
def getNeighbors():
    global isConnecting
    pos_list = []
    neighbor=[[0]*N for i in range(N)]
    for i in range(N):
        epuck=robot.getFromDef("epuck"+str(i))
        pos=epuck.getPosition()
        pos_list.append(pos[0:3]) 
        for j in range(i):
            dist=getDistance(pos_list[j],pos_list[i])
            if dist < max(commRanges[i],commRanges[j]):
                neighbor[j][i]=1
                neighbor[i][j]=1
    if neighbor[customData]:
        isConnecting = True
    else:
        isConnecting = False              
    return neighbor[customData]

def updateNeighbors():
    global oldNeighbors
    newNeighbors = getNeighbors()
    diffNeighbors = [1 if a ==1 and b==0 else 0 for a, b in zip(newNeighbors, oldNeighbors)]
    oldNeighbors = newNeighbors
    return diffNeighbors
       
def Diffusing():
    global threadCurrentlyRunning
    global STATE

    STATE = 1                     #更改状态为1，Explore
    if not threadCurrentlyRunning:     
        threadCurrentlyRunning = True
        try:
            thread.start_new_thread(WaitForDecision,("Thread-1",))           
        except Exception as e:
            print("threading error")
            print(e)

def WaitForDecision(threadName):
    global threadCurrentlyRunning
    global STOPLOOP
    global consensusReached
    try:
        result = getResult()
        if result != "":
            if "end" != result:
                print("id=%d,C:Response from Server:%s" %(customData,result))
                resultList=eval(result) #
                if resultList[0] == True and consensusReached == False:
                    print("consensusReached is %s for robot %2d" % (str(resultList[0]),customData))
                    f = open("/home/luo/all_result.txt", "a+")
                    f.write(str(resultList[1:]))
                    f.write('\n')
                    f.close()
                    f = open("/home/luo/tmp_result.txt", "a+")
                    f.write('epuck' + str(customData) + '\n')
                    f.close()
                    consensusReached = True
            else:
                print("id=%d,C:end..."% customData)
                f = open("/home/luo/tmp_result.txt", "a+")
                f.write('epuck' + str(customData) + '\n')
                f.close()     
                STOPLOOP = True     
    except Exception as e:
        traceback.print_exc()
    threadCurrentlyRunning = False

def getResult():
    global res
    try:
        res = s.recv(1024,0x40).decode()
        res = res.split("#")[1].strip("~")
    except Exception as e:
        res = ""
    finally:
        return res

##################################
####       Main Process       ####         
##################################    
if __name__ == '__main__':      
    f = open("/home/luo/commRange.txt","r")
    commRangetxt = int(f.readline())
    commRange = commRangetxt * commRangetxt / 100
    f.close() 
    commRanges =[1 if _ in ranger_robots else 0.25 for _ in range(N)] 
    speed_max = 5
    if customData in ranger_robots: 
        speed_max = 7.5
   
    commRange = commRanges[customData]

    #global variable
    black_count=0
    white_count=0
    object_pos_list=[]
    vote =()
    vote_id = 0
    opinion=0
    quality=random.random()
    direction = 0
    # remainingsyncTime = math.ceil(20 / timeFactor)
    remainingsyncTime=-1
    remainingTime = math.ceil(numpy.random.exponential(LAMDA))
    remainingExplorationTime = math.ceil(sigma+100/timeFactor)
    votes =[]
    STATE = 1
    threadCurrentlyRunning = False 
    res = ""      
    STOPLOOP = False
    voteFlag = False
    syncFlag = False
    consensusReached = False
    isConnecting = False 
    neighbors = [0]*N
    oldNeighbors = [0]*N
    current_color=0
    last_front_obstacle = 0
    last_right_obstacle = 0
    last_left_obstacle = 0
    action_time = 0
    doMoveBack = 0
    # 关联并使能传感器设备
    ps_sensor = []
    for i_ in range(sensor_num):
        ps_sensor.append(robot.getDevice(ps_sensor_name[i_]))
        ps_sensor[i_].enable(timestep)  
    # 关联camera
    camera_time_step = timestep
    camera = robot.getDevice('camera')
    camera.enable(camera_time_step)  
    #关联LED
    led_name=['led1','led3','led5','led7']
    led_num=4
    led=[]
    for i_ in range(led_num):
        led.append(LED(led_name[i_]))     
    # 关联电机设备
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    # 设置电机运行模式
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    #连接服务端
    IP = "127.0.0.1"
    PORT=9955+customData
    try:
        socket.setdefaulttimeout(10)
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 96)
        s.connect((IP, PORT))   
    except Exception as e:
        print("connection failed")
        print(e) 
        supvisornode = robot.getFromDef("epuck%s"%customData)
        supvisornode.restartController()  
    else:
        print("connection success")     
    finally:   
        ##################################
        ####        MainLoop          ####         
        ##################################       
        step = 0
        while robot.step(timestep) != -1:
            try:
                step = step + 1
                termination_time_ticks = termination_time_ticks -1
                if step > sigma+1:
                    ConnectAndListen() #get neighbors and sync
                
                TurnLeds()         #Turn Leds On
                
                Move()
    
                       
                if STATE == 1:  
                    Explore()      #explore and estimate 
                elif STATE == 2:
                    Diffusing()    #vote
            
                RandomWalk()
           
                if STOPLOOP:
                    stop()
                    break
            except Exception as e:
                print(traceback.print_exc())
                      
        s.close()
        total_steps = step
        total_seconds = timestep * total_steps / 1000.0
        print("total time is " + str(total_seconds) + "seconds") 