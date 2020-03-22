import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

# 绘制最初的位置 和 终点
# with open("data_orca.txt", "r") as f:
#     data  = f.readline()
# print(data)
# hh
# 3d画图
with open("data_orca.txt", "r") as f:
    datas = f.readlines()
stepnum = len(datas)
##创建125个nparray
agent_num = 1000
datashow = []
for i in range(agent_num):
    datashow.append(np.empty((3,1)))

for data in datas:

    data = data.split( )
    for i in range(len(data)):
        data[i] = float(data[i])
    data = np.array(data)

    # add element
    for i in range(int (len(data)/3)):
        # print(i)
        col = np.array([ data[3*i], data[3*i+1], data[3*i+2] ])
        col = col.reshape(3,1)
        datashow[i] = np.hstack((datashow[i], col))
for i in range(agent_num):
    datashow[i] = np.delete(datashow[i], 0, axis=1)
# print(datashow[0].shape)
# print(datashow)

"""
============
3D animation
============

A simple example of an animated plot... In 3D!
"""



def Gen_RandLine(length, dims=2):
    """
    Create a line using a random walk algorithm

    length is the number of points for the line.
    dims is the number of dimensions the line has.
    """
    lineData = np.empty((dims, length))
    lineData[:, 0] = np.random.rand(dims)   # 初始化起点
    for index in range(1, length):
        # scaling the random numbers by 0.1 so
        # movement is small compared to position.
        # subtraction by 0.5 is to change the range to [-0.5, 0.5]
        # to allow a line to move backwards.
        step = ((np.random.rand(dims) - 0.5) * 0.1)  # 步长
        # 下一步的位置
        lineData[:, index] = lineData[:, index - 1] + step
    return lineData   # 返回一个shape为（3,25）的数组,3维坐标25帧


def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Fifty lines of random 3-D lines  (长为50的数组，每个元素为shape为3,25的ndarray，最后实际效果就是50条路径)
# data = [Gen_RandLine(25, 3) for index in range(50)]
data = datashow
# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()
lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data] # 每条路径的起始点

limited = 108
# Setting the axes properties
ax.set_xlim3d([-limited, limited])
ax.set_xlabel('X')

ax.set_ylim3d([-limited, limited])
ax.set_ylabel('Y')

ax.set_zlim3d([-limited, limited])
ax.set_zlabel('Z')

ax.set_title('3D ORCA ZHX')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, stepnum, fargs=(data, lines),
                                   interval=2.5, blit=False)

plt.show()
