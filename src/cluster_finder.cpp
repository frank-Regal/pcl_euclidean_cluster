#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h> // include centroid header


// source: https://pcl-docs.readthedocs.io/en/latest/pcl/doc/tutorials/content/cluster_extraction.html

class EuclideanClusters
{
    public:
        EuclideanClusters()
        {
            sub = nh.subscribe<sensor_msgs::PointCloud2> ("/vaultbot/robot_manager/robots/vaultbot/ouster/points", 1, &EuclideanClusters::ProcessPointCloud, this);
            cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud_topic", 1);
            RobotEffPosition.x() = 0.8f;
            RobotEffPosition.y() = -0.1f;
            RobotEffPosition.z() = 0.01f;
            IsThereACloseCentroid = false;
        }

        void PublishCentroid(const Eigen::Vector3f& centroid)
        {
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped msg;
            msg.child_frame_id = "os_sensor";
            msg.header.frame_id = "centroid";
            msg.header.stamp = ros::Time(0);
            msg.transform.translation.x = centroid[0];
            msg.transform.translation.y = centroid[1];
            msg.transform.translation.z = centroid[2];
            msg.transform.rotation.x = 0;
            msg.transform.rotation.y = 0;
            msg.transform.rotation.z = 0;
            msg.transform.rotation.z = 1;
            br.sendTransform(msg);
        }

        void ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.1); // 2cm
            ec.setMinClusterSize (50);
            ec.setMaxClusterSize (110);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);
            

            float min_distance {1000000};
            float max_distance {2.00};
            
            int j = 0;
            int min_index {0};
            int num_clusters = (int) cluster_indices.size();
            std::cout << "Num Clusters: " << num_clusters << std::endl;
            

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
   
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                

                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
                }
                cloud_cluster_vec.push_back(cloud_cluster);

                // Compute centroid of closest cluster
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_cluster_vec[j], centroid);

                Eigen::Vector3f centroid_point = centroid.head<3>();
                float this_distance = (centroid_point - RobotEffPosition).norm();
                
                cloud_cluster_vec[j]->width = cloud_cluster_vec[j]->points.size ();
                cloud_cluster_vec[j]->height = 1;
                cloud_cluster_vec[j]->is_dense = true;

                if (this_distance < min_distance)
                {   
                    min_distance = this_distance;
                    min_index = j;

                    std::cout << "\n" << std::endl;
                    std::cout << "[NEW MIN CLUSTER] Cluster Number: " << min_index <<"; PointCloud representing the Cluster: " << cloud_cluster_vec[j]->points.size () << " data points." << std::endl;
                    // Print centroid coordinates
                    std::cout << "Centroid: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << std::endl;
                    std::cout << "Norm Distance: " << this_distance << std::endl;
                }
                
                j++;
                //std::cout << "Iter: " << j << std::endl;

                if (j == num_clusters)
                {
                    std::cout << "Final Vec" << std::endl;
                    pcl::toROSMsg(*cloud_cluster_vec[min_index], cluster_msg);
                    cloud_cluster_vec.clear();
                    cluster_msg.header.frame_id = "os_sensor";
                    cluster_msg.header.stamp = ros::Time(0);
                    cluster_pub.publish(cluster_msg);
                    //PublishCentroid(centroid_point);
                }
                
            }
            

            
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher cluster_pub;
        ros::Publisher centroid_tf_pub;
        sensor_msgs::PointCloud2 cluster_msg;
        Eigen::Vector3f RobotEffPosition;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_vec;
        bool IsThereACloseCentroid;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_finder");

  EuclideanClusters euclidean_clusters;

  std::cout << "Clustering Cloud" << std::endl;

  ros::spin();

  return 0;
}