#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"readpcl");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("./src/data/curvenoise_inliers.pcd", *cloud) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        std::cout << "    " << cloud->points[i].x
                    << " "    << cloud->points[i].y
                    << " "    << cloud->points[i].z << std::endl;
        // std::cout << cloud->sensor_orientation_ << std::endl;        
    }

    ROS_INFO("完成！");
    
    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//创建一个显示窗口
    // viewer.showCloud(cloud);					//显示点云
    // while(!viewer.wasStopped())
    // {  
    // }

	pcl::visualization::PCLVisualizer viewer("viewer");
	std::cout << "初始的姿态矩阵\n" << viewer.getViewerPose().matrix() << std::endl;

	viewer.addCoordinateSystem(0.5);
	viewer.addPointCloud(cloud);
	viewer.setCameraPosition(0, 3, 0, 0, -1, 0, 0, 0, 1); //视点 方向 上方向 

	std::cout << "加入相机参数以后\n" <<viewer.getViewerPose().matrix() << std::endl;

	viewer.spin();


    return (0);
}
// PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>


// ros::Publisher pub;

// void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
// {
// // Create a container for the data.
// sensor_msgs::PointCloud2 output;
// // Do data processing here...
// output = *input;
// // Publish the data.
// pub.publish(output);
// }





// int main(int argc, char** argv)
// {
// // Initialize ROS
// ros::init (argc, argv, "my_pcl_tutorial");
// ros::NodeHandle nh;
// // Create a ROS subscriber for the input point cloud
// ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
// // Create a ROS publisher for the output point cloud
// pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
// // Spin
// ros::spin();
// }
