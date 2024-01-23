import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('/Users/felichri/Documents/AnonAuton/anon_auton_ws/bags/2024-01-22-18-49-30.bag')

# get the list of topics
print(b.topic_table)

# get all the messages of type velocity
data = b.message_by_topic("statistics")
print(data)
veldf = pd.read_csv(data)
# plt.plot(veldf['Time'], veldf['linear.x'])

# # quickly plot velocities
# b.plot_vel(save_fig=True)

# # you can animate a timeseries data
# bagpy.animate_timeseries(veldf['Time'], veldf['linear.x'], title='Velocity Timeseries Plot')
