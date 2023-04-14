import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import serial
from serial.tools.list_ports import comports
import threading

df = pd.DataFrame()
running = True
fig = plt.figure()
ax = fig.add_subplot(1,1,1)

def animate(n):
    global df
    if df.shape[1] != 0 and df.shape[0] != 0:
        fig.cla()
        df.plot(x=df.columns[0],y=df.columns[1:], kind='line', ax=ax)

# end animate

def find_port():
    for port in comports():
        if "V5 User Port" in port.description:
            V5port = port.device
            print("\n" + "V5 found (" + V5port + ")")
            return V5port
    
    print("V5 not found")
    return 0
# end find_port

def collect_data(ser):
    global df
    while running:
            line = ser.readline().decode('ascii')
            # if data needs to be graphed
            if "graph_data" in line:
                line = ser.readline().decode('ascii')[6:].strip('\n')
                columns = line.split('|')[0].split(',')
                data = map(float, line.split('|')[1].split(','))
                data = pd.DataFrame([data], columns=columns)

                # concat data to dataframe
                df = pd.concat([df, data], ignore_index=True)

                # print data
                #print("\n" + data.to_string(index=False))
# end collect_data


# main

# find V5 port
port = find_port()
if port == 0:
    exit()

# open serial port
ser = serial.Serial(port, 115200, timeout=0.1)
print("Starting data collection...\n")
# start data collection thread
collect_thread = threading.Thread(target=collect_data, args=(ser,))
collect_thread.start()

# start animation
ani = animation.FuncAnimation(fig, animate, interval=10)
# show plot
plt.show()
print("Stopping data collection...")

# stop data collection
running = False
collect_thread.join()