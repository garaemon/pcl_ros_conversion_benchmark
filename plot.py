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
def plot_data(y_labels, labels, title):
  figure(title, figsize=[14, 8])
  plt.xlabel("the number of the points")
  plt.ylabel("time to convert(sec)")
  x = data[0][1:]
  for i in range(len(y_labels)):
    ylabel = y_labels[i]
    first_labels = [data[j][0] for j in range(len(data))]
    ys = data[first_labels.index(ylabel)][1:]
    label = labels[i]
    plot(x, ys, label=label)
  legend(loc="best")
  savefig("figures/" + title + ".png")

plot_data([
  "ROS -> PCL2 (XYZRGB/XYZ)",
  "ROS -> PCL2 (XYZRGBNORMAL/XYZRGB)",
  ], [
    "ROS(XYZRGB) -> PCL2(XYZRGB)",
    "ROS(XYZRGBNORMAL) -> PCL2(XYZRGBNORMAL)"
    ], "ROS -> PCL2")
    
           
plot_data([
  "ROS -> PCL (XYZRGB/XYZ)",
  "ROS -> PCL (XYZRGB/XYZRGB)",
  "ROS -> PCL (XYZRGBNORMAL/XYZ)",
  "ROS -> PCL (XYZRGBNORMAL/XYZRGB)",
  "ROS -> PCL (XYZRGBNORMAL/XYZNORMAL)",
  "ROS -> PCL (XYZRGBNORMAL/XYZRGBNORMAL)"
  ], [
    "ROS(XYZRGB) -> PCL(XYZ)",
    "ROS(XYZRGB) -> PCL(XYZRGB)",
    "ROS(XYZRGBNORMAL) -> PCL(XYZ)",
    "ROS(XYZRGBNORMAL) -> PCL(XYZRGB)",
    "ROS(XYZRGBNORMAL) -> PCL(XYZNORMAL)",
    "ROS(XYZRGBNORMAL) -> PCL(XYZRGBNORMAL)"
    ], 
    "ROS -> PCL")
plot_data([
  "PCL -> PCL2 (XYZRGB/XYZ)",
  "PCL -> PCL2 (XYZRGBNORMAL/XYZ)",
  ], [
    "PCL(XYZRGB) -> PCL2(XYZRGB)",
    "PCL(XYZRGBNORMAL) -> PCL2(XYZRGBNORMAL)",
    ], "PCL -> PCL2")

plot_data([
  "PCL -> ROS (XYZRGB/XYZ)",
  "PCL -> ROS (XYZRGBNORMAL/XYZ)",
  ], [
    "PCL(XYZRGB) -> ROS(XYZRGB)",
    "PCL(XYZRGBNORMAL) -> ROS(XYZRGBNORMAL)",
  ], "PCL -> ROS")

plot_data([
  "PCL2 -> ROS (XYZRGB/XYZ)",
  "PCL2 -> ROS (XYZRGBNORMAL/XYZ)",
  ], [
    "PCL2(XYZRGB) -> ROS(XYZRGB)",
    "PCL2(XYZRGBNORMAL) -> ROS(XYZRGBNORMAL)",
    ], "PCL2 -> ROS")

plot_data([
  "PCL2 -> PCL (XYZRGB/XYZ)",
  "PCL2 -> PCL (XYZRGB/XYZRGB)",
  "PCL2 -> PCL (XYZRGBNORMAL/XYZ)",
  "PCL2 -> PCL (XYZRGBNORMAL/XYZRGB)",
  "PCL2 -> PCL (XYZRGBNORMAL/XYZNORMAL)",
  "PCL2 -> PCL (XYZRGBNORMAL/XYZRGBNORMAL)"
  ], [
    "PCL2(XYZRGB) -> PCL(XYZ)",
    "PCL2(XYZRGB) -> PCL(XYZRGB)",
    "PCL2(XYZRGBNORMAL) -> PCL(XYZ)",
    "PCL2(XYZRGBNORMAL) -> PCL(XYZRGB)",
    "PCL2(XYZRGBNORMAL) -> PCL(XYZNORMAL)",
    "PCL2(XYZRGBNORMAL) -> PCL(XYZRGBNORMAL)"
    ], 
    "PCL2 -> PCL")


show()
