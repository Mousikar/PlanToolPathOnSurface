#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int
 main (int argc, char** argv)
{
    setlocale(LC_ALL,"");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("./src/data/planenoise.pcd", *cloud) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
    /************************************************************************************
     创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
    即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
    ***********************************************************************************/
    // 设置滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);            //设置输入点云
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-0.001, 0.001);        //设置在过滤字段的范围
    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内 是否保存滤波的限制范围内的点云，默认为false，保存限制范围内点云，true时候是相反。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
    std::cerr << "Cloud after filtering: " << std::endl;   //打印
    for (size_t i = 0; i < 20; ++i)//< cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;
    std::cerr << "滤波前的size" << cloud->points.size () << std::endl;
    std::cerr << "滤波后的size" << cloud_filtered->points.size () << std::endl;
	//保存pcd文件
	pcl::io::savePCDFileASCII("./src/data/planenoisefilter.pcd", *cloud_filtered);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//创建一个显示窗口
    viewer.showCloud(cloud_filtered);					//显示点云
    while(!viewer.wasStopped())
    {  
    }

    return (0);
}