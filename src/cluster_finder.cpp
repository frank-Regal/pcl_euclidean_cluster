#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class EuclideanClusters
{
    public:
        EuclideanClusters()
        {
            sub = nh.subscribe<sensor_msgs::PointCloud2> ("/vaultbot/robot_manager/robots/vaultbot/ouster/points", 1, &EuclideanClusters::callback, this);
        }

        void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
        {
            std::string frame_name = cloud_msg->header.frame_id;
            std::cout << "\n" << frame_name << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);
            std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_finder");

  EuclideanClusters euclidean_clusters;

  std::cout << "Clustering Cloud" << std::endl;

  ros::spin();

  return 0;
}