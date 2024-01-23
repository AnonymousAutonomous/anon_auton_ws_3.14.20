import bagpy
from bagpy import bagreader
import pandas as pd
import os
import matplotlib.pyplot as plt

folder = os.path.dirname(__file__)
b = bagreader(folder + '/2024-01-22-19-32-06.bag')

# get the list of topics
print(b.topic_table)

csvfiles = []
for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

print(csvfiles)
# print(csvfiles[0])
# data = pd.read_csv(csvfiles[0])

# get all the messages of type velocity
data = b.message_by_topic("statistics")
print(data)
df = pd.read_csv(data)
df.head(20)
plt.plot(df['Time'], df['node_pub'])

plt.show()
# # quickly plot velocities
# b.plot(save_fig=True)

# # you can animate a timeseries data
# bagpy.animate_timeseries(veldf['Time'], veldf['linear.x'], title='Velocity Timeseries Plot')
