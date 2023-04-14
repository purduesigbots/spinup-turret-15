"""
Various methods of drawing scrolling plots.
"""

from time import perf_counter

import numpy as np

import pyqtgraph as pg

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('pyqtgraph example: Scrolling Plots')

# 2) Allow data to accumulate. In these examples, the array doubles in length
#    whenever it is full. 
win.nextRow()
p1 = win.addPlot()
# Use automatic downsampling and clipping to reduce the drawing load
p1.setDownsampling(mode='peak')
p1.setClipToView(True)
curve1 = p1.plot()
curve2 = p1.plot()

data1 = np.empty(100)
ptr1 = 0

def update1():
    global data1, ptr1
    data1[ptr1] = np.random.normal()
    ptr1 += 1
    if ptr1 >= data1.shape[0]:
        tmp = data1
        data1 = np.empty(data1.shape[0] * 2)
        data1[:tmp.shape[0]] = tmp
    curve1.setData(data1[:ptr1])
    curve2.setData(data1[:ptr1]+7)


# 3) Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100
# Remove chunks after we have 10
maxChunks = 10
startTime = perf_counter()
win.nextRow()
p2 = win.addPlot()
p2.setLabel('bottom', 'Time', 's')
p2.setXRange(-10, 0)
curves = []
data2 = np.empty((chunkSize+1,2))
ptr2 = 0

def update2():
    global p2, data2, ptr2, curves
    now = perf_counter()
    for c in curves:
        c.setPos(-(now-startTime), 0)
    
    i = ptr2 % chunkSize
    if i == 0:
        curve = p2.plot()
        curves.append(curve)
        last = data2[-1]
        data2 = np.empty((chunkSize+1,2))        
        data2[0] = last
        while len(curves) > maxChunks:
            c = curves.pop(0)
            p2.removeItem(c)
    else:
        curve = curves[-1]
    data2[i+1,0] = now - startTime
    data2[i+1,1] = np.random.normal()
    curve.setData(x=data2[:i+2, 0], y=data2[:i+2, 1])
    ptr2 += 1


# update all plots
def update():
    update1()
    update2()
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

if __name__ == '__main__':
    pg.exec()

'''import pandas as pd
import matplotlib.pyplot as plt  
import matplotlib.animation as animation
import time
import threading
import random

#df = pd.read_csv('data.csv',delimiter=',',encoding='utf-8',engine='python')
df = pd.DataFrame()
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
running = True

def collect_data():
    global df
    while running:
        numbers = [df.shape[0], random.randint(0,100), random.randint(0,100)]
        data = pd.DataFrame([numbers], columns=['time','1','2'])
        df = pd.concat([df, data], ignore_index=True)
        time.sleep(0.01)

def animate(n):
    global df
    if df.shape[1] >= 2 and df.shape[0] >= 1:
        plt.cla()
        df.plot(x=df.columns[0],y=df.columns[1:], kind='line', ax=ax)

data_thread = threading.Thread(target=collect_data)
data_thread.start()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()

running = False
data_thread.join()'''
