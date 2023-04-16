//  a-shape 曲面重构(凹包算法扩展)   pcd  转化为 ply
 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
 
using namespace std;
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("./src/data/planenoisefilter.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> cavehull; 
	cavehull.setInputCloud(cloud);        
	cavehull.setAlpha(0.02);            
	vector<pcl::Vertices> polygons;       
	cavehull.reconstruct(*surface_hull, polygons);// 重建面要素到点云
 
	pcl::PolygonMesh mesh;
	cavehull.reconstruct(mesh);// 重建面要素到mesh
	pcl::io::saveOBJFile("./src/data/object_mesh.obj", mesh);
	cerr << "Concave hull has: " << surface_hull->points.size()
		<< " data points." << endl;
 
	//保存网格图
	pcl::io::savePLYFile("./src/data/result.ply", mesh);
 
 
	// pcl::PCDWriter writer;
	// writer.write("../ph_alpha.pcd", *surface_hull, false);
 
 
	// 	//___可视化重建结果___
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	// viewer->setBackgroundColor(0, 0, 0);        //  设置背景色为黑色
	// viewer->addPolygonMesh(mesh, "my");
	// viewer->addCoordinateSystem(1.0);          // 建立空间直角坐标系
	// viewer->initCameraParameters();                // 初始化相机参数
	// while (!viewer->wasStopped()){
	// 	viewer->spinOnce(100);                                // 显示
	// 	// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }
	// // 可视化
	// pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("hull"));
	// viewer->setWindowName("alshape曲面重构");
	// viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
	// viewer->spin();
 
	return (0);
}
 