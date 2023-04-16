#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);			//待滤波点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//滤波后点云

	///读入点云数据
	cout << "->正在读入点云..." << endl;
	pcl::PCDReader reader;
	reader.read("./src/data/curvenoise.pcd", *cloud);
	cout << "\t\t<读入点云信息>\n" << *cloud << endl;

	///均匀采样
	cout << "->正在均匀采样..." << endl;
	pcl::UniformSampling<pcl::PointXYZ> us;	//创建滤波器对象
	us.setInputCloud(cloud);				//设置待滤波点云
	us.setRadiusSearch(0.05f);				//设置滤波球体半径
	us.filter(*cloud_filtered);				//执行滤波，保存滤波结果于cloud_filtered

	///保存下采样点云
	cout << "->正在保存采样点云..." << endl;
	pcl::PCDWriter writer;
	writer.write("./src/data/curvenoisefilter.pcd", *cloud_filtered, true);
	cout << "\t\t<保存点云信息>\n" << *cloud_filtered << endl;

	//================================= 滤波前后对比可视化 ================================= ↓

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("滤波前后对比"));

	/*-----视口1-----*/
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); //设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
	viewer->setBackgroundColor(0, 0, 0, v1); //设置背景颜色，0-1，默认黑色（0，0，0）
	viewer->addText("befor_filtered", 10, 10, "v1_text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "befor_filtered_cloud", v1);

	/*-----视口2-----*/
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("after_filtered", 10, 10, "v2_text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "after_filtered_cloud", v2);

	/*-----设置相关属性-----*/
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor_filtered_cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "befor_filtered_cloud", v1);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after_filtered_cloud", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "after_filtered_cloud", v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	//================================= 滤波前后对比可视化 ================================= ↑

	return 0;
}
