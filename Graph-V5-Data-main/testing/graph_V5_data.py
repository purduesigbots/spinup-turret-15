from dash import Dash, dcc, html, Input, Output
import plotly.express as px
import pandas as pd
import time
import serial
import serial.tools.list_ports
import threading

app = Dash(__name__)
df = pd.DataFrame()
#df = pd.read_csv('data.csv',delimiter=',',encoding='utf-8',engine='python')

app.layout = html.Div([
    html.H4('V5 Data'),
    html.Div(id='live-update-text'),
    dcc.Graph(
        id = 'live-update-graph'),
    dcc.Interval(
        id = 'graph-update',
        interval = 100,
        n_intervals = 0
    )
])

@app.callback(
    Output('live-update-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph(n):
    if df.shape[1] >= 2:
        plot = px.scatter(df,x=df.columns[0],y=df.columns[1:])
        return plot
# end update_graph

def find_port():
    for port in serial.tools.list_ports.comports():
        if "V5 User Port" in port.description:
            V5port = port.device
            foundV5 = True
            print("\n" + "V5 found (" + V5port + ")")
            return V5port
    
    print("V5 not found")
    return 0
# end find_port

def connect_collect():
    port = find_port()

    if port == 0:
        return False

    global df

    try:
        # open serial port
        ser = serial.Serial(port, 115200, timeout=0.1)
        print("Starting data collection...\n")

        while True:
            line = ser.readline().decode('ascii')
            # if data needs to be graphed
            if "graph_data" in line:
                line = ser.readline().decode('ascii')[6:].strip('\n')
                columns = line.split('|')[0].split(',')
                data = line.split('|')[1].split(',')
                data = pd.DataFrame([data], columns=columns)

                # assign data to dataframe
                df = pd.concat([df, data], ignore_index=True)

                # print data
                print()
                print(data.to_string(index=False))

    except KeyboardInterrupt:
        print("\n" + "Serial port closed")
        if input("Save data? (y/n): ") == "y":
            df.to_csv("data.csv")
            print("Data saved to data.csv")
        return False
    except IndexError:
        print("\n" + "ERROR: Incorrect Formatting")
        print("(\"name,...,name|data,...,data\")")
        return False
    except serial.serialutil.SerialException:
        print("\n" + "ERROR: Lost Connection")
        if input("Would you like to reconnect? (y/n): ") == "y":
            return True
        elif input("Save data? (y/n): ") == "y":
            df.to_csv("data.csv")
            print("Data saved to data.csv")
            return False
    except Exception as e:
        print("\n" + str(e))
        return False
    finally:
        ser.close()
        print(df)
# end connect_collect

if __name__ == '__main__':
    #app.run_server(debug=True)
    connect_collect()