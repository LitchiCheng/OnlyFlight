import serial,re

sr = serial.Serial("COM10",115200)
r = re.compile("[|]|[\s]")

import matplotlib.pyplot as plt
import numpy as np

gx=[]
gy=[]
gz=[]
ax=[]
ay=[]
az=[]

num=0   #计数
plt.ion()    # 开启一个画图的窗口进入交互模式，用于实时更新数据
# plt.rcParams['savefig.dpi'] = 200 #图片像素
# plt.rcParams['figure.dpi'] = 200 #分辨率
plt.rcParams['figure.figsize'] = (10, 5)        # 图像显示大小
plt.rcParams['font.sans-serif']=['SimHei']   #防止中文标签乱码，还有通过导入字体文件的方法
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['lines.linewidth'] = 0.5   #设置曲线线条宽度
while num<100:
    # 获取数据
    data_str = sr.readline()
    data_str = data_str.decode("utf-8")
    x = r.split(data_str)
    print(x)
    plt.clf()    #清除刷新前的图表，防止数据量过大消耗内存
    plt.suptitle("MPU6050",fontsize=30)             #添加总标题，并设置文字大小
	
    gx.append(float(x[0]))       
    agraphic=plt.subplot(6,1,1)
    agraphic.set_title('gx')   
    plt.plot(gx,'g-') 
	
    gy.append(float(x[1]))
    bgraghic=plt.subplot(6, 1, 2)
    bgraghic.set_title('gy')
    bgraghic.plot(gy,'g-')
    
    gz.append(float(x[2]))
    bgraghic=plt.subplot(6, 1, 3)
    bgraghic.set_title('gz')
    bgraghic.plot(gz,'g-')
    
    ax.append(float(x[3]))
    bgraghic=plt.subplot(6, 1, 4)
    bgraghic.set_title('ax')
    bgraghic.plot(ax,'g-')
    #图表2
    ay.append(float(x[4]))
    bgraghic=plt.subplot(6, 1, 5)
    bgraghic.set_title('ay')
    bgraghic.plot(ay,'g-')
    #图表2
    az.append(float(x[5]))
    bgraghic=plt.subplot(6, 1, 6)
    bgraghic.set_title('az')
    bgraghic.plot(az,'g-')

    plt.pause(0.03)     #设置暂停时间，太快图表无法正常显示

plt.ioff()       # 关闭画图的窗口，即关闭交互模式
plt.show()       # 显示图片，防止闪退
