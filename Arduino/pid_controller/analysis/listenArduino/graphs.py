import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import csv

rootdir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(rootdir, 'data')

def graphData(Kp, Ki, Kd, setpoint, ffwd, start_time, filename, df):
    df = pd.DataFrame(df, columns = ['timestamp', 'input', 'output'])
    df = df.astype({"timestamp": int, "input": float, "output": float})
    # change to time since start time, in minutes
    df['timestamp'] - int(start_time)
    df['timestamp'] = df['timestamp'].div(60000).round(2)

    print(df)

    f1 = plt.figure()

    plt.plot(df['timestamp'], df['input']) 
    plt.axhline(y=float(setpoint), color='r', linestyle='-')
    plt.title(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}\nINPUT setpoint: {setpoint}, feedfwd: {ffwd}")

    # plt.show()
    plt.savefig(f'data/graphs/{filename}_input.pdf')  

    f2 = plt.figure()

    plt.plot(df['timestamp'], df['output']) 
    plt.axhline(y=float(setpoint), color='r', linestyle='-')
    plt.title(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}\nOUTPUT setpoint: {setpoint}, feedfwd: {ffwd}")

    # plt.show()
    plt.savefig(f'data/graphs/{filename}_output.pdf')  

def getDfFromCsv(filename):
    path = os.path.join(data_dir, filename)
    with open(path, 'r', newline='') as f:
        csv_data = [x for x in list(csv.reader(f, delimiter=',')) if x != []]
        i_of_init = 0

        # Kp, Ki, Kd, setpoint, ffwd = 0

        for i, row in enumerate(csv_data):
            if 'setpoint' in row:
                i_of_init = i
                print(csv_data[i + 1])
                start_time = csv_data[i + 1][0]
                Kp = csv_data[i + 1][1]
                Ki = csv_data[i + 1][2]
                Kd = csv_data[i + 1][3]
                setpoint = csv_data[i + 1][4]
                ffwd = csv_data[i + 1][5]
                
        print(Kp, Ki, Kd, setpoint, ffwd)
        print(i_of_init)

        df = csv_data[i_of_init + 3:]
        return (Kp, Ki, Kd, setpoint, ffwd, start_time, df)

for filename in os.listdir(data_dir):
    if filename.endswith('.csv'):
        Kp, Ki, Kd, setpoint, ffwd, start_time, df = getDfFromCsv(filename)
        graphData(Kp, Ki, Kd, setpoint, ffwd, start_time, filename[:-4], df)