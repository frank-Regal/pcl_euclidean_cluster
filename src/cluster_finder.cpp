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

class EuclideanClusters
{
    public:
        EuclideanClusters()
        {
            sub = nh.subscribe<sensor_msgs::PointCloud2> ("/vaultbot/robot_manager/robots/vaultbot/ouster/points", 1, &EuclideanClusters::callback, this);
            cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud_topic", 1);
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
            vg.setLeafSize (0.1f, 0.1f, 0.1f);
            vg.filter (*cloud_filtered);
            std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::PCDWriter writer;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);

            int i=0, nr_points = (int) cloud_filtered->points.size ();
            while (cloud_filtered->points.size () > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud (cloud_filtered);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                {
                    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                    break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers);
                extract.setNegative (false);

                // Get the points associated with the planar surface
                extract.filter (*cloud_plane);
                //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_f);
                *cloud_filtered = *cloud_f;
            }

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.5); // 2cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
                }
            
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

                pcl::toROSMsg(*cloud_cluster, cluster_msg);
                cluster_msg.header.frame_id = "os_sensor";
                cluster_msg.header.stamp = ros::Time(0);
                cluster_pub.publish(cluster_msg);
                
                //std::stringstream ss;
                //ss << "cloud_cluster_" << j << ".pcd";
                //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
                j++;
            }
            
            
            


        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher cluster_pub;
        sensor_msgs::PointCloud2 cluster_msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_finder");

  EuclideanClusters euclidean_clusters;

  std::cout << "Clustering Cloud" << std::endl;

  ros::spin();

  return 0;
}