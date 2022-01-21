import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import csv

rootdir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(rootdir, 'data', 'csvs')

def graphData(Kp, Ki, Kd, setpointA, setpointB, ffwda, ffwdb, start_time, filename, df):
    df = pd.DataFrame(df, columns = ['timestamp', 'inputA', 'outputA', 'a_adjust', 'inputB', 'outputB', 'b_adjust'])
    print(df)
    df = df.replace(to_replace='None', value=np.nan).dropna()
    
    df = df.astype({"timestamp": int, "inputA": float, "outputA": float, "a_adjust": float, "inputB": float, "outputB": float, "b_adjust": float})
    # change to time since start time, in minutes
    df['timestamp'] - int(start_time)
    df['timestamp'] = df['timestamp'].div(60000).round(2)

    print(df)

    fig, axs = plt.subplots(2, sharex=True)
    for ax in fig.get_axes():
        ax.label_outer()

    axs[0].set_title("INPUT")
    axs[0].plot(df['timestamp'], df['inputA'], color='b') 
    axs[0].plot(df['timestamp'], df['inputB'], color='g') 
    axs[0].axhline(y=float(setpointA), color='c', linestyle='-')
    axs[0].axhline(y=float(setpointB), color='r', linestyle='-')

    axs[1].set_title("OUTPUT")
    axs[1].plot(df['timestamp'], df['outputA'], color='b') 
    axs[1].plot(df['timestamp'], df['outputB'], color='g') 
    
    fig.suptitle(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}\n setpointA: {setpointA}, setpointB: {setpointB}, feedfwdA: {ffwda}, feedfwdB: {ffwdb}")

    # plt.show()
    plt.savefig(f'data/graphs/{filename}.pdf')  


def getDfFromCsv(filename):
    path = os.path.join(data_dir, filename)
    with open(path, 'r', newline='') as f:
        csv_data = [x for x in list(csv.reader(f, delimiter=',')) if x != []]
        i_of_init = 0

        print(csv_data)

        # Kp, Ki, Kd, setpoint, ffwd = 0

        for i, row in enumerate(csv_data):
            if 'setpointA' in row:
                i_of_init = i
                print(csv_data[i + 1])
                start_time = csv_data[i + 1][0]
                KpA = csv_data[i + 1][1]
                KiA = csv_data[i + 1][2]
                KdA = csv_data[i + 1][3]
                setpointA = csv_data[i + 1][4]
                ffwdA = csv_data[i + 1][5]
                KpB = csv_data[i + 1][6]
                KiB = csv_data[i + 1][7]
                KdB = csv_data[i + 1][8]
                setpointB = csv_data[i + 1][9]
                ffwdB = csv_data[i + 1][10]
                
        print(KpA, KiA, KdA, setpointA, setpointB, ffwdA, ffwdB)

        df = csv_data[i_of_init + 3:]
        print(df[:3])
        return (KpA, KiA, KdA, setpointA, setpointB, ffwdA, ffwdB, start_time, df)

for filename in os.listdir(data_dir):
    if filename.endswith('.csv'):
        Kp, Ki, Kd, setpointA, setpointB, ffwdA, ffwdB, start_time, df = getDfFromCsv(filename)
        graphData(Kp, Ki, Kd, setpointA, setpointB, ffwdA, ffwdB, start_time, filename[:-4], df)