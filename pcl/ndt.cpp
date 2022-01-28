/*
NDT算法点云配准
xiaochen wang
2021/06
*/
#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <time.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr read_point_clouds(string const &file_path); // 读取点云数据（pcd）
void visulizer(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);// 点云可视化

int main()
{
	
	// 读取点云数据（目标点集，待配准点集），显示点的个数
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = read_point_clouds("../data/l_0.pcd");
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = read_point_clouds("../data/l_1.pcd");
	cout << "Load " << target_cloud->size() << " data points from target cloud1." << endl;
	cout << "Load " << input_cloud->size() << " data points from target cloud2." << endl;
	visulizer(target_cloud, input_cloud); //初始读入数据可视化,关闭后执行下一步

	// 对cloud2进行下采样，降低点云数量，提高运行效率
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point(new pcl::PointCloud<pcl::PointXYZ>); //建立一个filtered_point，存储滤波（下采样）后的点云数据
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter; // 建立一个approximate_voxel_filter
	approximate_voxel_filter.setLeafSize(0.4, 0.4, 0.4); //设置体素大小
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_point); //滤波（降采样）后的点云数据
	cout << "Filtered cloud contains " << filtered_point->size() << " data points from input cloud." << endl;

	// 初始化NDT变换参数
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt; // 建立一个ndt
	ndt.setTransformationEpsilon(0.01); //ε（两个连续变换之间允许的最大差值），判断优化过程是否已经收敛。对[x, y, z, roll, pitch, yaw]以m和rad为单位的增量允许最小值,低于这个值，则停止
	ndt.setStepSize(0.1); //0.1牛顿法优化的最大步长，more-thuente线搜索设置最大步长
	ndt.setResolution(1.0); //格网化时的大小
	ndt.setMaximumIterations(35); //优化迭代的次数，当达到35次迭代，或者收敛到阈值ε时，停止迭代（防止在错误的方向运算太多时间），超出迭代次数则停止

	ndt.setInputSource(filtered_point); //待配准点集
	ndt.setInputTarget(target_cloud); //目标参考点集

	// 初始化变换关系p
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ()); // 欧拉角，绕Z轴旋转
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0); //初始化平移参数
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix(); //初始化变换关系p（* 运算符在这里是两个变换的串联）
	
	cout << "The initial transformation matrix:" << endl;
	cout << init_guess << endl << endl;

	
	cout << "NDT processing..." << endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>); //存储变换后的点云数据
	ndt.align(*output_cloud, init_guess); // init_guess为变换初值
	cout << "Normal distribution transformation has converged: " << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << endl;

	Eigen::Matrix4f optimized_trans = ndt.getFinalTransformation();
	cout << "The optimized transformation matrix:" << endl;
	cout << optimized_trans << endl << endl;

	// 对滤波前的数据进行变换，保存配准后的点云数据，输出为pcd格式
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation()); //使用计算的变换参数，对采样前的输入数据进行变换，输出到output_cloud
	pcl::io::savePCDFileASCII("../data/output_point_clouds.pcd", *output_cloud); // 输出变换后的点云数据

	cout << "Point cloud registration completed! "  << endl;
		
	//点云结果可视化
	visulizer(target_cloud, output_cloud);

	getchar();
	return 0;
}

// 读取点云数据（pcd）
pcl::PointCloud<pcl::PointXYZ>::Ptr read_point_clouds(string const &file_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *my_cloud) == -1) //判断是否能够成功读入pcd数据
	{
		cout << "Couldn't read the pcd file: " << file_path << endl;
		return nullptr;
	}

	return my_cloud;
}

// 点云可视化
void visulizer(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
	// 初始化3D Viewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);

	//设置点云颜色(目标点云，红色)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

	//设置点云颜色(目标点云，绿色)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");

	//开始可视化
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();

	//此while循环保持窗口一直处于打开状态，并且按照规定时间刷新窗口，直到关闭可视化窗口。
	//wasStopped()判断显示窗口是否已经被关闭，spinOnce()叫消息回调函数，作用其实是设置更新屏幕的时间
	//this_thread::sleep()在线程中调用sleep()
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

