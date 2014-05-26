#!/usr/bin/env python

from pylab import *
import sys

CSV_FILE = sys.argv[1]

print "reading", CSV_FILE
data = []
with open(CSV_FILE, "r") as f:
  legends = f.readline().split(",")
  for i in range(len(legends)):
    data.append([legends[i].strip()])           #fill with empty array
  print "legends", legends
  line = f.readline()
  while line:
    linedata = line.split(",")
    for i in range(len(linedata)):
      if linedata[i] != '\n':
        data[i].append(float(linedata[i]))
    line = f.readline()


# plotting
def plot_data(y_labels, title):
  figure(title)
  x = data[0][1:]
  for ylabel in y_labels:
    labels = [data[i][0] for i in range(len(data))]
    plot(x, data[labels.index(ylabel)][1:], label=ylabel)
  legend(loc="best")


plot_data(["ROS -> PCL2 (XYZRGB/XYZ)",
           "ROS -> PCL2 (XYZRGB/XYZRGB)",
           "ROS -> PCL2 (XYZRGBNORMAL/XYZ)",
           "ROS -> PCL2 (XYZRGBNORMAL/XYZRGB)",
           "ROS -> PCL2 (XYZRGBNORMAL/XYZNORMAL)",
           "ROS -> PCL2 (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "ROS -> PCL2")
plot_data(["ROS -> PCL (XYZRGB/XYZ)",
           "ROS -> PCL (XYZRGB/XYZRGB)",
           "ROS -> PCL (XYZRGBNORMAL/XYZ)",
           "ROS -> PCL (XYZRGBNORMAL/XYZRGB)",
           "ROS -> PCL (XYZRGBNORMAL/XYZNORMAL)",
           "ROS -> PCL (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "ROS -> PCL")
plot_data(["PCL -> PCL2 (XYZRGB/XYZ)",
           "PCL -> PCL2 (XYZRGB/XYZRGB)",
           "PCL -> PCL2 (XYZRGBNORMAL/XYZ)",
           "PCL -> PCL2 (XYZRGBNORMAL/XYZRGB)",
           "PCL -> PCL2 (XYZRGBNORMAL/XYZNORMAL)",
           "PCL -> PCL2 (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "PCL -> PCL2")
plot_data(["PCL -> ROS (XYZRGB/XYZ)",
           "PCL -> ROS (XYZRGB/XYZRGB)",
           "PCL -> ROS (XYZRGBNORMAL/XYZ)",
           "PCL -> ROS (XYZRGBNORMAL/XYZRGB)",
           "PCL -> ROS (XYZRGBNORMAL/XYZNORMAL)",
           "PCL -> ROS (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "PCL -> ROS")
plot_data(["PCL2 -> ROS (XYZRGB/XYZ)",
           "PCL2 -> ROS (XYZRGB/XYZRGB)",
           "PCL2 -> ROS (XYZRGBNORMAL/XYZ)",
           "PCL2 -> ROS (XYZRGBNORMAL/XYZRGB)",
           "PCL2 -> ROS (XYZRGBNORMAL/XYZNORMAL)",
           "PCL2 -> ROS (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "PCL2 -> ROS")
plot_data(["PCL2 -> PCL (XYZRGB/XYZ)",
           "PCL2 -> PCL (XYZRGB/XYZRGB)",
           "PCL2 -> PCL (XYZRGBNORMAL/XYZ)",
           "PCL2 -> PCL (XYZRGBNORMAL/XYZRGB)",
           "PCL2 -> PCL (XYZRGBNORMAL/XYZNORMAL)",
           "PCL2 -> PCL (XYZRGBNORMAL/XYZRGBNORMAL)"
           ], "PCL2 -> PCL")


show()
