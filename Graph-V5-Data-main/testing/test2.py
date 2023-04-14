import pyqtgraph as pg
import numpy as np
import pandas as pd

df = pd.read_csv('data.csv',delimiter=',',encoding='utf-8',engine='python')
win = pg.GraphicsLayoutWidget(show=True)
win.nextRow()
p_all = win.addPlot()
p_all.setLabel('bottom', df.columns[0])
p_all.addLegend()

curves = []
print(df)
#plot data from arr
if (df.shape[1] - 1 > len(curves)):
    for i in range(len(curves), df.shape[1] - 1):
        curve = p_all.plot(x=df.iloc[:,0], y=df.iloc[:,i + 1], name=df.columns[i + 1], pen=(i,df.shape[1]))
        curves.append(curve)
else:
    for i in range(df.shape[1] - 1, len(curves)):
        curves[i].setData(x=df.iloc[:,0], y=df.iloc[:,i + 1])

for i in range(len(curves)):
    print(i)

if __name__ == '__main__':
    pg.exec()
    print("done")
