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

int main(int argc, char** argv)
{
  boost::mt19937 rng( static_cast<unsigned long>(time(0)) );
  boost::uniform_real<> range(0,1);
  boost::variate_generator< boost::mt19937, boost::uniform_real<> > mt(rng, range);

  // the size of point cloud
  // 0 ~ 1024*768 for every 1024
  for (size_t point_num = 640*480; point_num < 1024 * 768; point_num += 1024 * 10) {
    double pcl_to_pcl2, pcl2_to_ros, ros_to_pcl2, pcl2_to_pcl;
    double rgb_pcl_to_pcl2, rgb_pcl2_to_ros, rgb_ros_to_pcl2, rgb_pcl2_to_pcl;
    
    pcl::PointCloud<pcl::PointXYZRGB> rgb_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for (size_t i = 0; i < point_num; i++) {
      pcl::PointXYZRGB p;
      p.x = p.y = p.z = p.rgb = i;
      rgb_pcl_cloud.points.push_back(p);
    }
    for (size_t i = 0; i < point_num; i++) {
      pcl::PointXYZ p;
      p.x = p.y = p.z = i;
      pcl_cloud.points.push_back(p);
    }
    
    clock_t c1,c2;
    c1 = clock();

    // pcl::PointCloud -> pcl::PCLPointCloud2
    for (size_t i = 0; i < test_num; i++) {
      pcl::PCLPointCloud2 tmp;
      pcl::toPCLPointCloud2(pcl_cloud, tmp);
    }
    
    c2 = clock();
    pcl_to_pcl2 = (double)(c2-c1)/CLOCKS_PER_SEC / test_num;

    pcl::PCLPointCloud2 pcl_cloud2;
    pcl::toPCLPointCloud2(pcl_cloud, pcl_cloud2);

    
    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      sensor_msgs::PointCloud2 tmp;
      pcl_conversions::fromPCL(pcl_cloud2, tmp);
    }
    c2 = clock();
    pcl2_to_ros = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl_conversions::fromPCL(pcl_cloud2, ros_cloud);
    const sensor_msgs::PointCloud2 ros_cloud_const = ros_cloud;
    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      pcl::PCLPointCloud2 tmp;
      pcl_conversions::toPCL(ros_cloud_const, tmp);
    }
    c2 = clock();
    ros_to_pcl2 = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;

    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      pcl::PointCloud<pcl::PointXYZ> tmp;
      pcl::fromPCLPointCloud2(pcl_cloud2, tmp);
    }
    c2 = clock();
    pcl2_to_pcl = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;


    // rgb test
    c1 = clock();

    // pcl::PointCloud -> pcl::PCLPointCloud2
    for (size_t i = 0; i < test_num; i++) {
      pcl::PCLPointCloud2 tmp;
      pcl::toPCLPointCloud2(rgb_pcl_cloud, tmp);
    }
    
    c2 = clock();
    rgb_pcl_to_pcl2 = (double)(c2-c1)/CLOCKS_PER_SEC / test_num;

    pcl::PCLPointCloud2 rgb_pcl_cloud2;
    pcl::toPCLPointCloud2(rgb_pcl_cloud, rgb_pcl_cloud2);

    
    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      sensor_msgs::PointCloud2 tmp;
      pcl_conversions::fromPCL(rgb_pcl_cloud2, tmp);
    }
    c2 = clock();
    rgb_pcl2_to_ros = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;

    sensor_msgs::PointCloud2 rgb_ros_cloud;
    pcl_conversions::fromPCL(rgb_pcl_cloud2, rgb_ros_cloud);
    const sensor_msgs::PointCloud2 rgb_ros_cloud_const = rgb_ros_cloud;
    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      pcl::PCLPointCloud2 tmp;
      pcl_conversions::toPCL(rgb_ros_cloud_const, tmp);
    }
    c2 = clock();
    rgb_ros_to_pcl2 = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;

    c1 = clock();
    for (size_t i = 0; i < test_num; i++) {
      pcl::PointCloud<pcl::PointXYZ> tmp;
      pcl::fromPCLPointCloud2(rgb_pcl_cloud2, tmp);
    }
    c2 = clock();
    rgb_pcl2_to_pcl = (double)(c2-c1)/CLOCKS_PER_SEC  / test_num;

    std::cout << point_num << "," << pcl_to_pcl2 << "," << pcl2_to_ros << "," << ros_to_pcl2 << "," << pcl2_to_pcl
              << "," << rgb_pcl_to_pcl2 << "," << rgb_pcl2_to_ros << "," << rgb_ros_to_pcl2 << "," << rgb_pcl2_to_pcl << std::endl;
  }
  return 0;
}
