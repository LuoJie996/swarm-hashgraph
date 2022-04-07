import socket
from time import time,sleep,strftime,localtime
import sys
import numpy as np
import math
import traceback
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
uuid_str = strftime("%Y-%m-%d-%H_%M_%S", localtime())
tmp_file_name = '/home/luo/blackblord_log.txt'
handler_info = logging.FileHandler(tmp_file_name, mode='a')
handler_info.setLevel(logging.INFO)
formatter = logging.Formatter('%(message)s')
handler_info.setFormatter(formatter)
logger.addHandler(handler_info)



class Node:
    def __init__(self, node_id):
        self.id = node_id
        self.blackboard_for_objects = []
        self.blackboard_for_grids = []
        self.startTime = time()
        self.vote_id = 0
        self.last_data = 0
        self.votes =[]
    def smart_contract(self, robot_id, x):
        pass

    def process(self, data):
        # print("%s,len=%d" % (data, len(data)))
        if len(data) < 114:
            data = ''
        else:
            data = data.split('#')[1]
            if len(data) < 113:
                data = ''
            else:
                data = eval(data.strip('~'))
                if data[3] == self.last_data:
                    data[2] = ''
                else:
                    self.last_data = data[3]
                    if data[2] == -1:
                        data[2] = ''
                    if self.id == 9:
                        self.votes.append(data[2])
                        # print(self.votes)
        return data
    def main(self):
        agent=Agent(self.id)
        new = ()
        while True:
            yield new
            data = agent.getData()
            payload = self.process(data)
            if payload:
                knowledge_diff = []
                if payload[2] and payload[2] != -2:
                    self.vote_id = payload[3]

                    if payload[2][0] and payload[2][0] not in blackboard_for_grids:
                        blackboard_for_grids.append(payload[2][0])
                    if payload[2][1] == 1:
                        if payload[2][0] not in blackboard_for_objects.keys():
                            blackboard_for_objects[payload[2][0]]=len(blackboard_for_grids)
                    diff = payload[2][2]-len(blackboard_for_grids)
                    if diff < 0:
                        knowledge_diff = blackboard_for_grids[diff:]
                    else:
                        knowledge_diff = []

                if len(blackboard_for_objects) == 5:
                    f = open("/home/luo/all_result.txt", "a+")
                    f.write("%s, %d, %d" % (str(blackboard_for_objects), len(blackboard_for_grids), self.vote_id))
                    f.write('\n')
                    f.close()
                    respond = "#end~"
                    agent.sendData(respond.encode())
                else:
                    t1 = time()
                    if t1 - self.startTime > 2:
                        if knowledge_diff:
                            knowledge_dict_response=[]
                            for i in knowledge_diff:
                                knowledge_dict_response.append(grids_dict[i])
                            # knowledge_dict_response =[]
                            # knowledge_dict_response =[i for i in range(450)]
                            respond = '#'+str(knowledge_dict_response)+'~'
                            # print("knowledge_diff: %d, %s" % (len(knowledge_dict_response),knowledge_dict_response))
                            agent.sendData(respond.encode())

class Agent:
    def __init__(self,node_id):
        self.id = node_id
        self.addr = (hostname, port + node_id)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建一个socket
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 228)
        self.srv.bind(self.addr)
        self.srv.listen(5)
        print("accept client")
        try:
            self.connect_socket, self.client_addr = self.srv.accept()
        except Exception as e:
            print(e)
        print("%2d,%s"%(self.id,str(self.client_addr)))


    def getData(self):
        try:
            data = self.connect_socket.recv(228).decode()
            print("robot %2d vote: %s", (self.id, data))
        except Exception as e:
            data = ''
            traceback.print_exc()
        return data

    def sendData(self, data):
        self.connect_socket.send(data)

def test(n_nodes):
    nodes =[Node(n) for n in range(n_nodes)]
    i = 0
    mains = [n.main() for n in nodes]
    for m in mains:
        print("start node %d" % i)
        next(m)
        i = i + 1
    r = 0

    while True:
        print('working node: %2i' % (r))
        next(mains[r])
        blackboardtxt =[]
        for i in blackboard_for_grids:
            blackboardtxt.append(grids_dict[i])
        # blackboardtxt = [i for i in range(450)]
        f=open('/home/luo/blackboard.txt','w')
        print(blackboardtxt,file=f)
        f.close()
        blackboardobjectstxt=[]
        for i in blackboard_for_objects:
            blackboardobjectstxt.append(grids_dict[i])
        f=open('/home/luo/blackboard_objects.txt','w')
        print(blackboardobjectstxt,file=f)
        f.close()
        print("blackboard_for_objects: %d, %s" % (len(blackboard_for_objects),str(blackboard_for_objects)))
        print("blackboard_for_grids_length: %d" % len(blackboard_for_grids))
        # sleep(0.01)
        r = r + 1
        if r == n_nodes:
            r = 0

grids_index = [30*i + j for i in range(30) for j in range(30)]
grids_points = [('{:.2f}'.format(-1.55+0.1*(i+1)),'{:.2f}'.format(-1.55+0.1*(j+1))) for i in range(30) for j in range(30)]
grids_dict = dict(zip(grids_points,grids_index))
print(grids_dict)
hostname = '127.0.0.1'
port = 9955
blackboard_for_objects = {}
blackboard_for_grids = []
start = time()
try:
    test(20)
except Exception as e:
    traceback.print_exc()
end = time()
time_diff = end - start
print(time_diff)