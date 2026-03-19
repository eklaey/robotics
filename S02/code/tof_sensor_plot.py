# run the script to plot the additional sensor responses 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# select data to plot
columns = [
    'tof',
    'ps0',
    'ps1',
#    'ps2',
#    'ps3',
#    'ps4',
#    'ps5',
    'ps6',
    'ps7',
    'placeholder' # here for ease of commenting in/out
     ]

# get data from CSV file
csv = pd.read_csv('../recordings/sensors.csv', index_col=0)

# drop last empty column
csv.drop(csv.columns[-1], axis=1, inplace = True)

# select columns to plot (removing placeholder)
new_csv = csv[columns[:-1]]
csv = new_csv

# plot single sensor one by one with subplots
csv.plot(subplots=True)
# save plot
plt.savefig('additional_sensors_single.png')
plt.show()

# plot all sensors on single plot
csv.plot()
# set the legend on right corner
plt.legend(loc='upper right')
plt.savefig('additional_sensors.png')
plt.show() 