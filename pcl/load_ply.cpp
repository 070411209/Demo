#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string>
    

int main(int argc, char* argv[]) {
    //argv[1] = "ricardo.ply";
    std::string incloudfile = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPLYFile<pcl::PointXYZI>(incloudfile, *cloud) == -1) {
        PCL_ERROR("Could not read file.\n");
        system("pause");
        return(-1);
    }
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    system("pause");
    return(0);
}
