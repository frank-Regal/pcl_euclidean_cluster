#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class PclEuclideanClustering
{
public:
  PclEuclideanClustering()
  {
    // Create a subscriber for the point cloud topic
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/vaultbot/robot_manager/robots/vaultbot/ouster/points", 1, &PclEuclideanClustering::cloudCallback, this);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_cloud_topic", 1);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Perform Euclidean Clustering Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.07); // Set the cluster tolerance (in meters)
    ec.setMinClusterSize(10); // Set the minimum cluster size
    ec.setMaxClusterSize(200); // Set the maximum cluster size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // Loop over the extracted clusters and do something with them
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        cluster->points.push_back(cloud->points[*pit]);
      }
    
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cluster->points.size () << " data points." << std::endl;
    
      // Do something with the extracted cluster, such as publish it on a new topic
      // For example:
      sensor_msgs::PointCloud2 cluster_msg;
      pcl::toROSMsg(*cluster, cluster_msg);
      cluster_msg.header.frame_id = "os_sensor";
      cluster_msg.header.stamp = ros::Time(0);
      cluster_pub_.publish(cluster_msg);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cluster_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_extractor");
  PclEuclideanClustering pcl_ec;
  std::cout << "Clustering Cloud" << std::endl;
  ros::spin();
  return 0;
}
