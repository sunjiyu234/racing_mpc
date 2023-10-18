import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg') 

with open('/home/sun234/racing_work/src/ltv_mpc/src/cur.txt', mode='r') as f1:
    list_1 = f1.readlines()
with open('/home/sun234/racing_work/src/ltv_mpc/src/xy.txt', mode='r') as f2:
    list_xy_lines = f2.readlines()
list_x_2, list_y_2 = [], []
for line in list_xy_lines:
    list_x_2.append(float(line.split(',')[0]))
    list_y_2.append(float(line.split(',')[1]))
list_2 = [float(list_i) for list_i in list_1]
fig1 = plt.figure(dpi = 80)
plt.plot(range(len(list_2)), list_2)
fig2 = plt.figure(dpi = 80)
plt.plot(list_x_2, list_y_2)
plt.show()
plt.savefig('/home/sun234/racing_work/src/ltv_mpc/src/cur.jpg')

