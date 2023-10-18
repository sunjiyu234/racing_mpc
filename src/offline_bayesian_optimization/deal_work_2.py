import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
matplotlib.use('TkAgg') 


x0 = -12.1517
y0 = 42.857
x_end = -3.7171
y_end = -6.0474
s = math.sqrt((x_end - x0) * (x_end - x0) + (y_end - y0) * ( y_end - y0))
if (s) > 10.0:
    num = int(s / 5.0)
with open('/home/sun234/racing_work/src/ltv_mpc/src/xy_new.txt', mode='w') as f1:
    for i in range(num - 1):
        x1 = x0 + (x_end - x0) * (i + 1) / (num)
        y1 = y0 + (y_end - y0) * (i + 1) / num
        f1.write('%f,%f\n'%(x1,y1))




