#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
    

void drawPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string titleName)
{
    pcl::visualization::PCLVisualizer viewer (titleName);
    int v (0);

    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v);

    viewer.addCoordinateSystem(0.5);

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_in_color_h (cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v1", v);

    viewer.addText (titleName, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v);

    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v);

    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

int main(int argc, char* argv[]) {
    //argv[1] = "ricardo.ply";
    std::string incloudfile = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_save;
    cloud_save = loadPointCloud(incloudfile);

    drawPointCloud(cloud_save, "user defined pointcloud");


    return(0);
}
