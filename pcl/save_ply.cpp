#include <iostream>

//点云需要的头文件
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
 
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

void savePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string outpath)
{
    std::cerr << "---- save path is :"<< outpath<< endl;
    //将string保存路径转为char*
    char *path = new char[outpath.size() +1];
    strcpy(path , outpath.c_str());
    std::cerr << "[save]Path is : " << path << " ." << std::endl;
	
    //写出点云图
    pcl::PLYWriter writer;
    writer.write(path, *cloud, true);
    std::cerr << "[save]PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

	if (pcl::io::loadPCDFile(filename, *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
		return 0;
	}

    std::cout<<"pointcloud size: "<<cloud->width<<" * "<<cloud->height << std::endl; 
    return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPlyPointCloud(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPLYFile<pcl::PointXYZI>(filename, *cloud) == -1) 
    {       
        PCL_ERROR("Couldnot read file.\n");
        return 0;
    }

    std::cout<<"pointcloud size: "<<cloud->width<<" * "<<cloud->height << std::endl; 
    return cloud;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
#if 1     
    cloud = loadPointCloud("/home/one/calib_data/kaiwo/lidar_gnss/lidar1/record_0.pcd");
#endif

#if 0   
    cloud = loadPlyPointCloud("/home/one/catkin_ws/src/lidar_align/results/lidar_points.ply");
#endif

    savePointCloud(cloud, "../save.ply");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_save;
    cloud_save = loadPlyPointCloud("/home/one/catkin_ws/src/lidar_align/results/a.ply"); // /home/one/catkin_ws/src/lidar_align/results/lidar_points.ply
    drawPointCloud(cloud_save, "user defined pointcloud");

	return 1;
}
