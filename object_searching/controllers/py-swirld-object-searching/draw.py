# code:utf-8  	Ubuntu
import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
from matplotlib.pyplot import MultipleLocator

import matplotlib.font_manager as mpt
zhfont=mpt.FontProperties(fname='/usr/share/fonts/custom/msyh.ttf') #显示中文字体
#导入数据
file = './log/Info_2022-01-07-12_41_43_log.txt'
a = np.loadtxt(file,skiprows=1)
print(a)
# 数组切片
x = a[:,0]  # 取第一列数据
y1 = a[:,1]  # 取第二列数据
y2 = a[:,2]
y3 = a[:,3]
# 画图
x_major_locator=MultipleLocator(10)
#把x轴的刻度间隔设置为1，并存在变量里
y_major_locator=MultipleLocator(500)
#把y轴的刻度间隔设置为10，并存在变量里
ax=plt.gca()
#ax为两条坐标轴的实例
ax.xaxis.set_major_locator(x_major_locator)
#把x轴的主刻度设置为1的倍数
ax.yaxis.set_major_locator(y_major_locator)
#把y轴的主刻度设置为10的倍数
#plt.xlim(0,300)
#把x轴的刻度范围设置为-0.5到11，因为0.5不满一个刻度间隔，所以数字不会显示出来，但是能看到一点空白
#plt.ylim(0,10000)


plt.plot(x,y1,label='confirmed') 
plt.plot(x,y2,label='received')
plt.plot(x,y3,label='all')
plt.legend()
ax.set_xlim(0,None)
ax.set_ylim(0,None)
# 保存图片  

plt.show()
