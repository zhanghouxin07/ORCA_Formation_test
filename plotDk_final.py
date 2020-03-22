import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

with open("final_Dk.txt", "r") as f:
    datas = f.readlines()

print(datas)
DK = []
for data in datas:
    DK.append(float(data))
cnt = []
for i in range(len( DK)):
    cnt.append((i+1))
plt.subplot (1 ,2 ,1)
plt.plot(cnt,DK)
plt.xlabel('t/s')
plt.ylabel('Dis_k/m')

with open("final_errk.txt", "r") as f:
    datas = f.readlines()

errk = []
for data in datas:
    errk.append(float(data))
cnt = []
for i in range(len( errk)):
    cnt.append((i+1))
plt.subplot (1 ,2 ,2)
plt.plot(cnt,errk)
plt.xlabel('t/s')
plt.ylabel('ERRk/m')

plt.show()