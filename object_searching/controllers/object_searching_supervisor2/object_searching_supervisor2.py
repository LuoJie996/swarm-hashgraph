#!/usr/bin/python3
# coding=UTF-8

from controller import Supervisor
import math,random
import socket
import os
import time
import _thread as thread
import logging

robot = Supervisor()
timestep =640
N = 20
group_number = 4
groups = {0:[],1:[],2:[],3:[]}
swarm =[]

for i in range(N):
    groups[i%4].append(i)
print(groups)    
for g in groups.keys():
    swarm.extend(groups[g])
print(swarm)    
f = open("/home/luo/commRanger.txt","r")
X = int(f.readline())
f.close()         
X = 4
ranger_robots=list(range(N)[:X])
print(ranger_robots)


# create the Robot instance.

commRange =[1 if _ in ranger_robots else 0.25 for _ in swarm] 

def init():
    USE_FLAT_TILE = False
    # obstacle_ratio = 0.40
    # obstacles=random.sample(range(900),int(900*obstacle_ratio))
    obstacles = [30*x + y for x in range(30) for y in range(30) if abs(y-15) <=2 or abs(15-x) <=2]
    # obstacles = []
    # objects =random.sample(set(range(900))-set(obstacles),int((900-len(obstacles))*0.1))
    object_ratio = 0.4
    field_1 = set([30*x + y for x in range(30) for y in range(30) if x >15 and y>15])-set(obstacles)
    field_2 = set([30*x + y for x in range(30) for y in range(30) if x <15 and y>15])-set(obstacles)
    field_3 = set([30*x + y for x in range(30) for y in range(30) if x <15 and y<15])-set(obstacles)
    field_4 = set([30*x + y for x in range(30) for y in range(30) if x >15 and y<15])-set(obstacles)
    ratio_1 = 0.04
    ratio_2 = 0.04
    ratio_3 = 0.04
    ratio_4 = 0.04
    objects_1 = random.sample(field_1,6)
    objects_2 = random.sample(field_2,6)
    objects_3 = random.sample(field_3,6)
    objects_4 = random.sample(field_4,6)
    # objects_4 = random.sample(field_4,int(len(field_4)*ratio_4))
    objects =list(set(objects_1) | set(objects_2) | set(objects_3) | set(objects_4))
    # objects=[232, 91, 651, 632]
    print("objects = %s" % str(objects))

    #a=[9 if i==0 else 8 if i==3 else 7 if i==5 else 0  for i in range(10) ]
    y_=[0.052 if i in ranger_robots else 0 for i in swarm]
    xz_1 = [(-1.55+0.1*(int(i/30)+1),-1.55+0.1*(i%30+1)) for i in field_1]
    xz_2 = [(-1.55+0.1*(int(i/30)+1),-1.55+0.1*(i%30+1)) for i in field_2]
    xz_3 = [(-1.55+0.1*(int(i/30)+1),-1.55+0.1*(i%30+1)) for i in field_3]
    xz_4 = [(-1.55+0.1*(int(i/30)+1),-1.55+0.1*(i%30+1)) for i in field_4]
    random.shuffle(xz_1)
    random.shuffle(xz_2)
    random.shuffle(xz_3)
    random.shuffle(xz_4)
    n=math.ceil(N/4)
    # xz = xz_1[0:n] + xz_2[0:n] + xz_3[0:n] + xz_4[0:n]
    xz = [(1.05, 0.95), (0.95, 0.55), (0.6500000000000001, 1.3500000000000003), (0.55, 1.45), (0.95, 0.44999999999999996), (-0.85, 1.45), (-1.15, 0.8500000000000003), (-0.65, 1.2500000000000002), (-1.15, 0.44999999999999996), (-0.65, 1.3500000000000003), (-0.25, -0.95), (-1.15, -0.44999999999999996), (-1.35, -0.65), (-1.45, -1.35), (-0.55, -0.25), (0.8500000000000003, -1.45), (1.1500000000000001, -0.25), (0.7500000000000002, -1.15), (0.8500000000000003, -0.85), (0.3500000000000001, -0.65)]
    print("xz = %s "% str(xz))
    # xlist = [i / 100.0 for i in range(-90,90,int(180/N))]
    # zlist = [i / 100.0 for i in range(-90,90,int(180/N))]
    rolist = [i / 100.0 for i in range(0,628,int(628/N))]
    rgblist = []
    rgblist = ['{:08b}'.format(i+1) if i in ranger_robots else "none" for i in swarm]
    
    # random.shuffle(zlist)
    random.shuffle(rolist)
    # print(xlist)
    # print(zlist)
    #importRobot(1,xlist[0],zlist[0])
    drawtiles(obstacles, objects, USE_FLAT_TILE)
    objects_points = [('{:.2f}'.format(-1.55+0.1*(i+1)),'{:.2f}'.format(-1.55+0.1*(j+1))) for i in range(30) for j in range(30) if 30*i+j in objects]
    f = open("/home/luo/all_result.txt", "a+")
    f.write(str(objects_points))
    f.write('\n')
    f.close()    
    print("=================DEBUG5=================")
    runs=readFile()
    print("=================DEBUG6=================")
    print(runs)
    if runs > 9:
        msg = 'echo Experiment is finished | mail -s "10 runs is finished" 2385943799@qq.com'
        #sendmail(msg)
        #changeFloor()
        # changeByzNum()
        # changeRange()
        #changeSpeeder()
        changeRanger()

    print("=================DEBUG1=================")
    #连接服务端
    IP="localhost"
    PORT=8888
    socket.setdefaulttimeout(5)
    s = socket.socket()
    try:
        for i in range(N):
            j = swarm.index(i)
            print(j)
            importRobot(i,xz[j][0],y_[j],xz[j][1],rolist[j],rgblist[j])
        s.connect((IP,PORT))
        msg='ready'
        msg=msg.encode()
        s.send(msg)

        while(True):
            print("=================DEBUG2=================")
            res = s.recv(1024).decode()
            print('-----------------------------------------------res:'+str(res))
            if res=="reset":
                print("=================DEBUG3=================")
                # for i in range(N):
                    # epuck=robot.getFromDef("epuck"+str(i))
                    # epuck.restartController()                
                print("==============================reset done")
                break   
    except Exception as e:
        print("connection failed")
    s.close
            
def resetAll():
   
    writeFile()
    #robot.worldReload()
    robot.simulationReset()
    # for i in range(N):
        # epuck_node=robot.getFromDef("epuck"+str(i))
        # epuck_node.restartController()
    supervisornode = robot.getFromDef("EnvSet_supervisor")
    supervisornode.restartController()

def changeSpeeder():
    f = open("/home/luo/Speeder.txt","r")
    speeder = int(f.readline())
    f.close()
    speederNew = speeder + 2
    f = open("/home/luo/Speeder.txt","w+")
    f.write(str(speederNew))
    f.close()
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_speeder"+str(speeder)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if speeder >= N:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'cat /home/luo/all_result.txt | mail -s "Speeder Experiment is finished" 2385943799@qq.com'
        sendmail(msg)    

def changeRanger():
    f = open("/home/luo/commRanger.txt","r")
    commRanger = int(f.readline())
    f.close()
    commRangerNew = commRanger + 1
    f = open("/home/luo/commRanger.txt","w+")
    f.write(str(commRangerNew))
    f.close()
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_ranger"+str(commRanger)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if commRanger >= 10:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'echo /home/luo/all_result.txt | mail -s "Ranger Experiment is finished" 2385943799@qq.com'
        sendmail(msg)
               
def changeRange():
    f = open("/home/luo/commRange.txt","r")
    commRange = int(f.readline())
    f.close()
    commRangeNew = commRange + 1
    f = open("/home/luo/commRange.txt","w+")
    f.write(str(commRangeNew))
    f.close()
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_range"+str(commRange)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if commRange >= 20:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'echo /home/luo/all_result.txt | mail -s "Experiment is finished" 2385943799@qq.com'
        sendmail(msg)
        
def changeByzNum():
    f = open("/home/luo/ByzNum.txt","r")
    ByzNum = int(f.readline())
    f.close()
    ByzNumNew = ByzNum + 1
    f = open("/home/luo/ByzNum.txt","w+")
    f.write(str(ByzNumNew))
    f.close()
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_ByzNum"+str(ByzNum)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if ByzNum >= 10:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'echo /home/luo/all_result.txt | mail -s "Experiment is finished" 2385943799@qq.com'
        sendmail(msg)
        
def changeSpeed():
    f = open("/home/luo/Speed.txt","r")
    speed = int(f.readline())
    f.close()
    speedNew = speed + 2
    f = open("/home/luo/Speed.txt","w+")
    f.write(str(speedNew))
    f.close()
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_speed"+str(speed)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if speed >= 20:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'cat /home/luo/all_result.txt | mail -s "Speed Experiment is finished" 2385943799@qq.com'
        sendmail(msg)         
    
def changeFloor():
    floor=robot.getFromDef("FloorImage")
    url=floor.getField("url")
    urlstr=url.getMFString(0)
    urllist=urlstr.split('.')
    print(urllist)
    ratio=int(urllist[1])
    urlstr=urllist[0]+"."+str(2+ratio)+"."+urllist[2]
    url.setMFString(0,urlstr)

    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SAVE RESULT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    try:        
        os.system("mv /home/luo/all_result.txt /home/luo/all_result_"+str(ratio)+".txt")
    except Exception as e:
        print("exception:" + e)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> CLEAR FILE2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    clearFile2()
    if ratio == 48:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> SendEmail >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        msg = 'echo /home/luo/all_result.txt | mail -s "Experiment is finished" 2385943799@qq.com'
        sendmail(msg)
def mail(threadName,msg):
    try:
        os.system(msg)
        print ("邮件发送成功")
    except smtplib.SMTPException:
        print ("Error: 无法发送邮件")     
def sendmail(msg):
    try:
        thread.start_new_thread(mail,("Thread-mail",msg,))           
    except Exception as e:
        print(e)

        

def clearFile():
    f2 = open("/home/luo/tmp_result.txt", "w")
    f2.write('')
    f2.close()              
           
def clearFile2():
    f2 = open("/home/luo/runs.txt", "w")
    f2.write('')
    f2.close()
    
def writeFile():
    f2 = open("/home/luo/runs.txt", "a+")
    f2.write('reset to next run \n')
    f2.close()   

def readFile():
    f2 = open("/home/luo/runs.txt")
    runs = len(f2.readlines())
    f2.close() 
    return runs    
    
def removeline():
    root = robot.getRoot()
    chFd = root.getField("children")

    while 1:
        count = chFd.getCount()
        #print(count)
        if count > 29:
            chFd.removeMF(count-1)
        else:
            break

def getNeighbors():
    pos_list = []
    neighbor=[[0]*N for i in range(N)]
    for i in range(N):
        epuck=robot.getFromDef("epuck"+str(i))
        pos=epuck.getPosition()
        pos_list.append(pos[0:3]) 
        for j in range(i):
            dist=getDistance(pos_list[j],pos_list[i])
            if dist < max(commRange[i],commRange[j]):
                neighbor[j][i]=1
                neighbor[i][j]=1              
    for r in range(N):
        for j in range(N):
            if neighbor[r][j] == 1:
                drawline(pos_list[r],pos_list[j])
                

            
def getDistance(pos1,pos2):
    dist=(pos1[0]-pos2[0])**2+(pos1[2]-pos2[2])**2
    return dist

def drawline(pos1,pos2):
    root = robot.getRoot()
    chFd = root.getField("children")
    #position = []
    line_String = """
                            DEF Line Shape{
                                appearance Appearance{
                                    material Material {
                                        diffuseColor 1 0 0
                                        emissiveColor 1 0 0
                                    }
                                }
                                geometry IndexedLineSet {
                                    coord Coordinate {
                                        point [ %f 0.054 %f, %f 0.054 %f ]
                                    }
                                    coordIndex [1 0]
                                }
                                isPickable FALSE
                            }
                            """ % (pos1[0], pos1[2], pos2[0], pos2[2])
    chFd.importMFNodeFromString(-1,line_String)
        
def importRobot(id,x,y,z,ro,rgb):
    root = robot.getRoot()
    chFd = root.getField("children")
    #position = []
    if rgb == "none":
        line_String = """
                                DEF epuck%d E-puck{
                                    translation %f %f %f
                                    rotation 0 1 0 %f
                                    name "e-puck%d"
                                    controller "object_searching2"
                                    customData "%d"
                                    supervisor TRUE
                                    version "2"
                                    camera_fieldOfView 0.5
                                    camera_width 48
                                    camera_height 48
                                    camera_antiAliasing TRUE
                                    camera_rotation 1 0 0 -1.46
                                }
                                """% (id,x,y,z,ro,id,id)
    else:
        line_String = """
                                DEF epuck%d E-puck2{
                                    translation %f %f %f
                                    rotation 0 1 0 %f
                                    name "e-puck%d"
                                    controller "object_searching2"
                                    customData "%d"
                                    supervisor TRUE
                                    version "2"
                                    camera_fieldOfView 0.5
                                    camera_width 48
                                    camera_height 48
                                    camera_antiAliasing TRUE
                                    camera_rotation 1 0 0 -1.46
                                    emissiveColor %s %s %s
                                    emissiveColor2 %s %s %s
                                }
                                """% (id,x,y,z,ro,id,id,rgb[-1],rgb[-2],rgb[-3],rgb[-4],rgb[-5],rgb[-6])
    chFd.importMFNodeFromString(-1,line_String)
    
def importTable():
    root = robot.getRoot()
    chFd = root.getField("children")
    #position = []
    line_String = """
                            DEF table Shape{
                                appearance Appearance{
                                    material Material {
                                        diffuseColor 0 0 0
                                        emissiveColor 0.2 0.2 0.2
                                    }
                                }
                                geometry IndexedFaceSet {
                                    coord Coordinate {
                                        point [ 1.01 -0.01 -1.01, -1.01 -0.01 -1.01, -1.01 -0.01 1.01, 1.01 -0.01 1.01, 0.8 -1 -0.8, -0.8 -1 -0.8, -0.8 -1 0.8, 0.8 -1 0.8 ]
                                    }
                                    coordIndex [ 3 7 4 0 -1  # face A, right
                                                 2 6 7 3 -1  # face B, back
                                                 1 5 6 2 -1  # face C, left
                                                 0 4 5 1 -1  # face D, front
                                                 7 6 5 4 -1  # face F, top
                                                 3 0 1 2 ] # face E, bottom
                                                  
                                }
                                isPickable FALSE
                            }
                            """ 
    chFd.importMFNodeFromString(-1,line_String)

def drawtiles(black_list, object_list, use_flat):
    root = robot.getRoot()
    chFd = root.getField("children")
    #position = []
    if use_flat:
        tiles_list = ["""
                                        Solid {
                                            translation %f 0.001 %f
                                            children [
                                                Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        emissiveColor %f %f %f
                                                    }
                                                    geometry Box {
                                                        size 0.1 0.001 0.1
                                                    }
                                                }
                                            ]
                                        }
                                        """ % (-1.55+0.1*(i+1),-1.55+0.1*(j+1),0.2,0.2,0.2) if 30*i+j in black_list else 
                                        """
                                        Solid {
                                            translation %f 0.001 %f
                                            children [
                                                Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        emissiveColor %f %f %f
                                                    }
                                                    geometry Box {
                                                        size 0.1 0.001 0.1
                                                    }
                                                }
                                            ]
                                        }
                                        """ % (-1.55+0.1*(i+1),-1.55+0.1*(j+1),1,1,1) for i in range(30) for j in range(30)]
    else:
        tiles_list = ["""
                                        Solid {
                                            translation %f 0.001 %f
                                            children [
                                                Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        emissiveColor %f %f %f
                                                    }
                                                    geometry Box {
                                                        size 0.1 0.001 0.1
                                                    }
                                                }
                                            ]
                                        }
                                        """ % (-1.55+0.1*(i+1),-1.55+0.1*(j+1),0,1,1) if 30*i+j in black_list else 
                                        """
                                        Solid {
                                            translation %f 0.001 %f
                                            children [
                                                Shape {
                                                    appearance Appearance {
                                                        
                                                        texture ImageTexture {
                                                            url "textures/parquetry/mines4.png"
                                                        }
                                                        
                                                    }
                                                    geometry Box {
                                                        size 0.1 0.001 0.1
                                                    }
                                                }
                                            ]
                                        }
                                        """ % (-1.55+0.1*(i+1),-1.55+0.1*(j+1)) if 30*i+j in object_list else
                                        """
                                        Solid {
                                            translation %f 0.001 %f
                                            children [
                                                Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        emissiveColor %f %f %f
                                                    }
                                                    geometry Box {
                                                        size 0.1 0.001 0.1
                                                    }
                                                }
                                            ]
                                        }
                                        """ % (-1.55+0.1*(i+1),-1.55+0.1*(j+1),1,1,1) for i in range(30) for j in range(30)]
    children_str=""
    for i in tiles_list:
        children_str = children_str + i
    line_String = """
                            DEF Floor_Tiles Solid{
                                children [
                                    %s
                                ]
                            }
                            """ % children_str
    chFd.importMFNodeFromString(-1,line_String)
    



    
# Main loop:
# - perform simulation steps until Webots is stopping the controller


    #time.sleep(3)

init()
clearFile()
start_time=robot.getTime()
oldtime = robot.getTime()
while robot.step(timestep) != -1:
    newtime = robot.getTime()
    timediff = newtime -oldtime
    #print(timediff)
    # removeline()
    # if timediff > 1:
        # oldtime = newtime
        # getNeighbors()



    filename = "/home/luo/tmp_result.txt"
    myfile = open(filename)
    lines = len(myfile.readlines())
    myfile.close()
    
    end_time=robot.getTime()
    total_time=end_time - start_time
    if(total_time > 2000):
        print("Exceeded max time")
        f = open("/home/luo/all_result.txt", "a+")
        f.write('Exit Time: ' + str(total_time) + '\n')
        s = "----------------Consensus NotReached! This experiment is time-exceeded-----------------"
        f.write(s)
        f.write('\n')
        f.close()
        resetAll()
        #robot.worldReload()
        
    if(lines >= N):   
        print("There are %d lines in %s" % (lines, filename))
        clearFile()
        #Save Exit Time
        f = open("/home/luo/all_result.txt", "a+")
        f.write('Exit Time: ' + str(total_time) + '\n')
        s = "----------------Consensus Reached! This experiment is finished-----------------"
        f.write(s)
        f.write('\n')
        f.close()
        resetAll()

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
