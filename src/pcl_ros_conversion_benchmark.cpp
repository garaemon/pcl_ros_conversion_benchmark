#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#include <boost/range/irange.hpp>
#include <iostream>

#include <boost/random.hpp>
#include <stdio.h>
#include <time.h>

const int test_num = 100;

#define TO_SEC(c1, c2) ((double)(c2-c1)/CLOCKS_PER_SEC  / test_num)

template <class OriginalPointType, class IntermediatePointType>
std::vector<double> benchmark(size_t point_num)
{
  // first generating the original input
  // we generate all the types first
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::PointCloud<OriginalPointType> pcl_cloud;
  pcl::PCLPointCloud2 pcl2_cloud;
  for (size_t i = 0; i < point_num; i++) {
    OriginalPointType p;
    pcl_cloud.points.push_back(p); // empty is OK, I beleive...
  }
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  pcl::toPCLPointCloud2(pcl_cloud, pcl2_cloud);
  const sensor_msgs::PointCloud2 ros_cloud_const = ros_cloud;
  clock_t c1, c2;
  
  // sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    pcl::PCLPointCloud2 tmp;
    pcl_conversions::toPCL(ros_cloud_const, tmp);
  }
  c2 = clock();
  double ros_to_pcl2 = TO_SEC(c1, c2);

  // pcl::PCLPointCloud2 -> pcl::PointCloud
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    pcl::PointCloud<IntermediatePointType> tmp;
    pcl::fromPCLPointCloud2(pcl2_cloud, tmp);
  }
  c2 = clock();
  double pcl2_to_pcl = TO_SEC(c1, c2);

  // pcl::PointCloud -> pcl::PCLPointCloud2
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    pcl::PCLPointCloud2 tmp;
    pcl::toPCLPointCloud2(pcl_cloud, tmp);
  }
  c2 = clock();
  double pcl_to_pcl2 = TO_SEC(c1, c2);

  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    sensor_msgs::PointCloud2 tmp;
    pcl_conversions::fromPCL(pcl2_cloud, tmp);
  }
  c2 = clock();
  double pcl2_to_ros = TO_SEC(c1, c2);

  // sensor_msgs::PointCloud2 -> pcl::PointCloud
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    pcl::PointCloud<IntermediatePointType> tmp;
    pcl::fromROSMsg(ros_cloud, tmp);
  }
  c2 = clock();
  double ros_to_pcl = TO_SEC(c1, c2);

  // pcl::PointCloud -> sensor_msgs::PointCloud2
  c1 = clock();
  for (size_t i = 0; i < test_num; i++) {
    sensor_msgs::PointCloud2 tmp;
    pcl::toROSMsg(pcl_cloud, tmp);
  }
  c2 = clock();
  double pcl_to_ros = TO_SEC(c1, c2);

  std::vector<double> ret;
  ret.push_back(ros_to_pcl2);
  ret.push_back(pcl2_to_pcl);
  ret.push_back(pcl_to_pcl2);
  ret.push_back(pcl2_to_ros);
  ret.push_back(ros_to_pcl);
  ret.push_back(pcl_to_ros);
  return ret;
}

int main(int argc, char** argv)
{
  boost::mt19937 rng( static_cast<unsigned long>(time(0)) );
  boost::uniform_real<> range(0,1);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > mt(rng, range);

  // the size of point cloud
  // 0 ~ 1024*768 for every 1024
  
  // legends
  std::vector<std::string> conversion_types;
  std::vector<std::string> point_types;
  conversion_types.push_back("ROS -> PCL2");
  conversion_types.push_back("PCL2 -> PCL");
  conversion_types.push_back("PCL -> PCL2");
  conversion_types.push_back("PCL2 -> ROS");
  conversion_types.push_back("ROS -> PCL");
  conversion_types.push_back("PCL -> ROS");
  point_types.push_back("XYZRGB/XYZ");
  point_types.push_back("XYZRGB/XYZRGB");
  point_types.push_back("XYZRGBNORMAL/XYZ");
  point_types.push_back("XYZRGBNORMAL/XYZRGB");
  point_types.push_back("XYZRGBNORMAL/XYZNORMAL");
  point_types.push_back("XYZRGBNORMAL/NORMAL");
  point_types.push_back("XYZRGBNORMAL/XYZRGBNORMAL");
  std::cout << "point num";
  for (size_t i = 0; i < conversion_types.size(); i++) {
    for (size_t j = 0; j < point_types.size(); j++) {
      std::cout << "," << conversion_types[i] << " (" << point_types[j] << ")";
    }
  }
  std::cout << std::endl;
  //std::cout << "point num,ROS -> PCL2 (XYZRGB/XYZ),ROS -> PCL2 (XYZRGB/XYZRGB),ROS -> PCL2 (XYZRGBNORMAL/XYZ)," << std::endl;
  for (size_t point_num = 640*480; point_num < 1024 * 768; point_num += 1024 * 10) {
    std::vector<double> xyzrgb_xyz = benchmark<pcl::PointXYZRGB, pcl::PointXYZ>(point_num);
    std::vector<double> xyzrgb_xyzrgb = benchmark<pcl::PointXYZRGB, pcl::PointXYZRGB>(point_num);
    std::vector<double> xyzrgbnormal_xyz = benchmark<pcl::PointXYZRGBNormal, pcl::PointXYZ>(point_num);
    std::vector<double> xyzrgbnormal_xyzrgb = benchmark<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(point_num);
    std::vector<double> xyzrgbnormal_xyznormal = benchmark<pcl::PointXYZRGBNormal, pcl::PointNormal>(point_num);
    std::vector<double> xyzrgbnormal_normal = benchmark<pcl::PointXYZRGBNormal, pcl::Normal>(point_num);
    std::vector<double> xyzrgbnormal_xyzrgbnormal = benchmark<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(point_num);
    std::cout << point_num << ",";
    for (size_t i = 0; i < xyzrgb_xyz.size(); i++) {
      std::cout << xyzrgb_xyz[i] << ",";
      std::cout << xyzrgb_xyzrgb[i] << ",";
      std::cout << xyzrgbnormal_xyz[i] << ",";
      std::cout << xyzrgbnormal_xyzrgb[i] << ",";
      std::cout << xyzrgbnormal_xyznormal[i] << ",";
      std::cout << xyzrgbnormal_normal[i] << ",";
      std::cout << xyzrgbnormal_xyzrgbnormal[i] << ",";
    }
    std::cout << std::endl;

  }
  return 0;
}
