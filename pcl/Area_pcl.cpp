#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;

	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
	}
	clicked_points_3d->height = 1;
	clicked_points_3d->width += indices.size();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	//每添加一次点云都要取一次别名，不然只能选择一次
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";
	//添加点云，并设置其显示半径
	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	// 保存点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("plane.pcd", *clicked_points_3d);
}

int main()
{
	if (pcl::io::loadPCDFile("../l_0.pcd", *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return 0;
	}
	viewer->addPointCloud(cloud, "point");//添加显示点云
	clicked_points_3d->width = 0;//初始化点云个数
	viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


  return 0;
}
