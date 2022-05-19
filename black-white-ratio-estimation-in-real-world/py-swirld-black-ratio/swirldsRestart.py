# coding=UTF-8
import os
import time
import _thread as thread
import socket
import traceback

s = socket.socket()         # 创建 socket 对象
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
host = "localhost"
port = 8888                # 设置端口
s.bind((host, port))        # 绑定端口
s.listen(10)                 # 等待客户端连接
print("server is ready to connect...")

data = "notready"
flag = True
def restartSwirlds(threadName):
	print("starting swirld")
	os.system("ps auxww | grep 'swirld_black_white_ratio_estimate_with_filter.py' | awk '{print $2}' | xargs kill -9")
	os.system("python3 swirld_black_white_ratio_estimate_with_filter.py") # block here
	print("#######################################")


def restartServer():
	try:
		print("try to start Thread")
		thread.start_new_thread(restartSwirlds,("Thread-1",)) #not block here
		print("Thread started")
	except Exception as e:
		traceback.print_exc()
		print("Error: unable to start thread")
	time.sleep(1)
	print("swirlds ready")
	return True


while(True):

	c,addr = s.accept()     # wait客户端连接
	print('连接地址：'+str(addr)) # connected
	#receive client's data
	try:
		data = c.recv(1024).decode()
		print(data)
		if data == "ready":
			#data = "notready"
			
			serverReady=restartServer()
			if serverReady:
				msg = "reset"
				msg = msg.encode()
				c.send(msg)
	except Exception as e:
		print(e)
	c.close()                # 关闭连接
s.close()

