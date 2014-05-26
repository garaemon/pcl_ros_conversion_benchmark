#!/bin/sh
gnuplot <<EOF
set datafile separator ","
set xlabel "number of the points"
set ylabel "time to convert (sec)"
plot "output.csv" using 1:2 with lines title 'ROS -> PCL2 (XYZRGB/XYZ)'
replot "output.csv" using 1:3 with lines title 'ROS -> PCL2 (XYZRGB/XYZRGB)'
replot "output.csv" using 1:4 with lines title 'ROS -> PCL2 (XYZRGBNORMAL/XYZ)'
replot "output.csv" using 1:5 with lines title 'ROS -> PCL2 (XYZRGBNORMAL/XYZRGB)'
replot "output.csv" using 1:6 with lines title 'ROS -> PCL2 (XYZRGBNORMAL/XYZNORMAL)'
replot "output.csv" using 1:7 with lines title 'ROS -> PCL2 (XYZRGBNORMAL/XYZRGBNORMAL)'
EOF
