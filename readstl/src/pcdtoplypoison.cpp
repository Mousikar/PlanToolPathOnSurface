 
//  泊松重建      pcd  转化为 ply
#include <iostream>
#include <string>
 
//点的类型的头文件
#include <pcl/point_types.h>
//点云文件IO（pcd文件和ply文件）
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//kd树
#include <pcl/kdtree/kdtree_flann.h>
//特征提取
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
 
int main(int argc, char** argv)
{
	//根据文件格式选择输入方式
	pcl::PointCloud < pcl:: PointXYZ> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
	pcl::io::loadPCDFile("./src/data/curvenoisefilter.pcd", *cloud) ;
		
	// ___ 法线估计 ___
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);  //10 
	n.compute(*normals); //计算法线，结果存储在normals中
 
	//___连接法线和坐标____
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
 
	//___泊松重建___
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
 
	pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	// pn.setDegree(2);   //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(6);     // 6 树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setMinDepth(2); 
	pn.setIsoDivide(6);  // 6 用于提取ISO等值面的算法的深度   
	pn.setSamplesPerNode(10); // 10 设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(3); // 3 设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//pn.setIndices();
 
	pn.setConfidence(false);
	pn.setManifold(false);    //是否添加多边形的重心，当多边形三角化时。
	pn.setOutputPolygons(false);  //是否输出多边形网格（而不是三角化移动立方体的结果）
 
	//设置搜索方法和输入点云
	pn.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	
	//___保存重建结果___
	//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;
	//执行重构
	pn.performReconstruction(mesh);
 
	//保存网格图
	pcl::io::savePLYFile("./src/data/result.ply", mesh);
 
	//___可视化重建结果___
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);        //  设置背景色为黑色
	viewer->addPolygonMesh(mesh, "my");
	viewer->addCoordinateSystem(1.0);          // 建立空间直角坐标系
	viewer->initCameraParameters();                // 初始化相机参数
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);                                // 显示
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
 
	return 0;
}