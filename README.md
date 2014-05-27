pcl_ros_conversion_benchmark
============================

benchmark of pcl-ros conversion

# classes
* `sensor_msgs::PointCloud2` pointcloud message type used in ROS
* `pcl::PointCloud<PointT>` pointcloud message type used in PCL
* `pcl::PCLPointCloud2` it's just a PCL version of `sensor_msgs::PointCloud2`. In order to be independent from ROS, PCL have their own pointcloud type
which is compatible with `sensor_msgs::PointCloud2`.

The problems comes from `PointT` template argument of `pcl::PointCloud`. When we convert `sensor_msgs::PointCloud2`
and `pcl::PCLPointCloud2` to `pcl::PointCloud<PointT>`, we can decrease the fields of the points.
For example, though `pcl::PCLPointCloud2` has rgb field, we can convert it to `pcl::PointCloud<PointXYZ>` only
with x, y and z fields.

# machine spec
```
$ cat /proc/meminfo
MemTotal:       65943128 kB
MemFree:        60777948 kB
Buffers:          520556 kB
Cached:          2017312 kB
SwapCached:            0 kB
Active:          1774876 kB
Inactive:        1948904 kB
Active(anon):    1186712 kB
Inactive(anon):      936 kB
Active(file):     588164 kB
Inactive(file):  1947968 kB
Unevictable:         188 kB
Mlocked:             188 kB
SwapTotal:      67075068 kB
SwapFree:       67075068 kB
Dirty:              9324 kB
Writeback:             0 kB
AnonPages:       1187512 kB
Mapped:           209560 kB
Shmem:              1704 kB
Slab:             486496 kB
SReclaimable:     374728 kB
SUnreclaim:       111768 kB
KernelStack:        6760 kB
PageTables:        36084 kB
NFS_Unstable:          0 kB
Bounce:                0 kB
WritebackTmp:          0 kB
CommitLimit:    100046632 kB
Committed_AS:    4339640 kB
VmallocTotal:   34359738367 kB
VmallocUsed:      445496 kB
VmallocChunk:   34325212796 kB
HardwareCorrupted:     0 kB
AnonHugePages:         0 kB
HugePages_Total:       0
HugePages_Free:        0
HugePages_Rsvd:        0
HugePages_Surp:        0
Hugepagesize:       2048 kB
DirectMap4k:      210992 kB
DirectMap2M:     7094272 kB
DirectMap1G:    59768832 kB
$ cat /proc/cpuinfo
vendor_id       : GenuineIntel
cpu family      : 6
model           : 45
model name      : Intel(R) Xeon(R) CPU E5-2687W 0 @ 3.10GHz
stepping        : 7
microcode       : 0x710
cpu MHz         : 1200.000
cache size      : 20480 KB
physical id     : 0
siblings        : 16
core id         : 0
cpu cores       : 8
apicid          : 0
initial apicid  : 0
fpu             : yes
fpu_exception   : yes
cpuid level     : 13
wp              : yes
flags           : fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge mca cmov pat pse36 clflush dts acpi mmx fxsr sse sse2 ss ht tm pbe syscall nx pdpe1gb rdtscp lm constant_tsc arch_perfmon pebs bts rep_good nopl xtopology nonstop_tsc aperfmperf pni pclmulqdq dtes64 monitor ds_cpl vmx smx est tm2 ssse3 cx16 xtpr pdcm pcid dca sse4_1 sse4_2 x2apic popcnt tsc_deadline_timer aes xsave avx lahf_lm ida arat epb xsaveopt pln pts dtherm tpr_shadow vnmi flexpriority ept vpid
bogomips        : 6200.54
clflush size    : 64
cache_alignment : 64
address sizes   : 46 bits physical, 48 bits virtual
power management:
...
```

# experiments


![PCL -> PCL2](figures/PCL -> PCL2.png)
![PCL -> ROS](figures/PCL -> ROS.png)
![ROS -> PCL2](figures/ROS -> PCL2.png)
![ROS -> PCL](figures/ROS -> PCL.png)
![PCL2 -> PCL](figures/PCL2 -> PCL.png)
![PCL2 -> ROS](figures/PCL2 -> ROS.png)

# conclusions
If you want to convert `sensor_msgs/PointCloud2` and `pcl::PCLPointCloud2` to `pcl::PointCloud`, you
should not decrease the fields of the points.
If the input pointcloud has x, y, z and rgb, you should convert it to `PointXYZRGB`.

For example, when we have a pointcloud from kinect (VGA and the fields are x, y, z and rgb),
it costs around 0.02 sec to convert the pointclouds from ROS to PCL and from PCL to ROS.
If your pointcloud processing takes 0.03sec (= 30fps), summation of the processing and conversion will be
0.05sec (= 20fps).

There is a huge *jump* around 700k points in several experiments. We need to dig this issue more to address
the reason but cache contamination is a potential reason of this problem.
