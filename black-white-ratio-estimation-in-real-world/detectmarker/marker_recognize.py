import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
import rospy
from std_msgs.msg import Int32MultiArray       

rospy.init_node('marker_recognize', anonymous = False)
rate = rospy.Rate(100)
pub = rospy.Publisher('/adjmat', Int32MultiArray, queue_size = 1) 

#with np.load('webcam_calibration_output.npz') as X:
#    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

#mtx =
#2946.48    0    1980.53
#0    2945.41    1129.25
#0    0    1
# mtx:
#  [[637.66160011   0.         300.51294796]
#  [  0.         637.65833456 242.20303585]
#  [  0.           0.           1.        ]]
# dist:
#  [[-0.04898693  0.40163146  0.00090572 -0.0017265  -0.79489243]]
mtx = np.array([
        [637.66160011,       0, 300.51294796],
        [      0, 637.65833456, 242.20303585],
        [      0,       0,       1],
        ])
#我的手机拍棋盘的时候图片大小是 4000 x 2250
#ip摄像头拍视频的时候设置的是 1920 x 1080，长宽比是一样的，
#ip摄像头设置分辨率的时候注意一下


dist = np.array( [-0.04898693, 0.40163146, 0.00090572, -0.0017265, -0.79489243] )

# video = "http://admin:admin@192.168.1.2:8081/"   # 手机ip摄像头
# 根据ip摄像头在你手机上生成的ip地址更改，右上角可修改图像分辨率
# video = "/home/luo/Videos/55.mp4"
video = 0

cap = cv2.VideoCapture(video)

w=1280
h=960
# 设置视频帧的宽和高
cap.set(cv2.CAP_PROP_FRAME_WIDTH,w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,h)
cap.set(cv2.CAP_PROP_BRIGHTNESS,155)
cap.set(cv2.CAP_PROP_CONTRAST,6)
lux=cap.get(cv2.CAP_PROP_CONTRAST)
print(lux)

# 获取视频总帧数和fps
count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
fps = cap.get(cv2.CAP_PROP_FPS)

# 视频编码格式
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

out = cv2.VideoWriter('/home/luo-m2/Videos/video_save.mp4', fourcc, fps, (w, h), True)

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
#num = 0

p1 = np.float32([[63,57], [1190,49], [1198,662], [75,679]])
p2 = np.float32([[0,0], [1279,0], [1279,719], [0,719]])
M = cv2.getPerspectiveTransform(p1,p2)
N=6
ranger_robots=[]
#通信距离，56对应10cm
commRanges =[560 if _ in ranger_robots else 336 for _ in range(N)]
def getDistance(pos1,pos2):
    dist=math.sqrt((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)
    return dist

while True:
    ret, frame = cap.read()
    # operations on the frame come here
    rows, cols, channels = frame.shape

    frame = cv2.warpPerspective(frame, M, (cols, rows))
    frame = cv2.flip(frame, -1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
    parameters =  aruco.DetectorParameters_create()
    '''
    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
    '''

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          parameters=parameters)
    aruco.drawDetectedMarkers(frame, corners)

    id_position = {0:(-10000,-10000),1:(-10000,-10000),2:(-10000,-10000),3:(-10000,-10000),4:(-10000,-10000),5:(-10000,-10000)}

#    if ids != None:
    if ids is not None:

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # Estimate pose of each marker and return the values rvet and tvec---different
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error

#        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #Draw Axis
#        aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

        print("before updating++++++++++")
        print(id_position)	
	
        for i in range(rvec.shape[0]):
            # aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            # aruco.drawDetectedMarkers(frame, corners)
        ###### DRAW ID #####


            print(corners[i])
            x_left = corners[i][0][0][0]
            x_right = corners[i][0][2][0]
            y_top = corners[i][0][0][1]
            y_bottom = corners[i][0][2][1]
            x_center = (x_left + x_right) / 2
            y_center = (y_top + y_bottom) / 2
            id_position[ids[i][0]]=(x_center, y_center)
            cv2.putText(frame, str(ids[i][0])+": "+str((x_center,y_center)), (int(x_center),int(y_center)), font, 1, (0,255,0),3, cv2.LINE_AA)
        print("after updating-----------")
        print(id_position)
        pos_list = []
        neighbor=[[0]*N for i in range(N)]
        for i in range(N):
            pos=id_position[i]
            pos_list.append(pos) 
            for j in range(i):
                dist=getDistance(pos_list[j],pos_list[i])
                if dist < max(commRanges[i],commRanges[j]):
                    neighbor[j][i]=1
                    neighbor[i][j]=1
                    cv2.line(frame, (int(pos_list[i][0]),int(pos_list[i][1])), (int(pos_list[j][0]),int(pos_list[j][1])), (0,255,0), 2)
        neighbors=[]
        for i in range(N):
            neighbors += neighbor[i]
        print(neighbors)
        a = Int32MultiArray()
        a.data= neighbors
        pub.publish(a)
    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow("frame",frame)
    out.write(frame)
    key = cv2.waitKey(33)

    if key == 27:         # 按esc键退出
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break

    if key == ord(' '):   # 按空格键保存
#        num = num + 1
#        filename = "frames_%s.jpg" % num  # 保存一张图像
        filename = str(time.time())[:10] + ".jpg"
        cv2.imwrite(filename, frame)
