#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "../l_0.pcd",
          *cloud) == -1) {
    PCL_ERROR("Couldn't read file rabbit.pcd\n");
    return (-1);
  }

  std::cout << "Loaded:" << cloud->width * cloud->height
            << " points from test_pcd.pcd with the following fields:"
            << std::endl;
  for (size_t i = 0; i < 10; ++i) {
    std::cout << "   " << cloud->points[i].x << "   " << cloud->points[i].y
              << "   " << cloud->points[i].z << "   " << std::endl;
  }

  std::cout << "----------- " << cloud->points.size() << " -------------------" << std::endl;
  return 0;
}
