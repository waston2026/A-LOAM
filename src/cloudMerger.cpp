// Point Cloud Merger for Multi-Sensor Fusion
// This node merges point clouds from 4 small FOV sensors into a single unified point cloud
// for use with LOAM-based SLAM algorithms
//
// Based on the requirements from paper section 3.2.3 regarding multi-sensor fusion

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class CloudMerger
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_merged_cloud_;
    tf::TransformListener tf_listener_;
    
    // Subscribers for 4 sensors
    ros::Subscriber sub_cloud1_;
    ros::Subscriber sub_cloud2_;
    ros::Subscriber sub_cloud3_;
    ros::Subscriber sub_cloud4_;
    
    // Storage for latest clouds from each sensor
    sensor_msgs::PointCloud2::ConstPtr latest_cloud1_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud2_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud3_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud4_;
    
    std::string target_frame_;
    bool use_transform_;
    double merge_timeout_;
    
public:
    CloudMerger() : nh_("~")
    {
        // Parameters
        nh_.param<std::string>("target_frame", target_frame_, "base_link");
        nh_.param<bool>("use_transform", use_transform_, true);
        nh_.param<double>("merge_timeout", merge_timeout_, 0.1);
        
        // Publisher for merged cloud
        pub_merged_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_cloud", 10);
        
        // Subscribers for 4 sensor topics
        sub_cloud1_ = nh_.subscribe("/sensor1/points", 10, &CloudMerger::cloud1Callback, this);
        sub_cloud2_ = nh_.subscribe("/sensor2/points", 10, &CloudMerger::cloud2Callback, this);
        sub_cloud3_ = nh_.subscribe("/sensor3/points", 10, &CloudMerger::cloud3Callback, this);
        sub_cloud4_ = nh_.subscribe("/sensor4/points", 10, &CloudMerger::cloud4Callback, this);
        
        ROS_INFO("Cloud Merger initialized");
        ROS_INFO("Target frame: %s", target_frame_.c_str());
        ROS_INFO("Use transform: %s", use_transform_ ? "true" : "false");
    }
    
    void cloud1Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        latest_cloud1_ = msg;
        tryMerge();
    }
    
    void cloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        latest_cloud2_ = msg;
        tryMerge();
    }
    
    void cloud3Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        latest_cloud3_ = msg;
        tryMerge();
    }
    
    void cloud4Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        latest_cloud4_ = msg;
        tryMerge();
    }
    
    void tryMerge()
    {
        // Check if we have received data from all sensors
        if (!latest_cloud1_ || !latest_cloud2_ || !latest_cloud3_ || !latest_cloud4_)
            return;
        
        // Check timestamps - ensure they're close enough
        ros::Time latest_time = latest_cloud1_->header.stamp;
        if (fabs((latest_cloud2_->header.stamp - latest_time).toSec()) > merge_timeout_ ||
            fabs((latest_cloud3_->header.stamp - latest_time).toSec()) > merge_timeout_ ||
            fabs((latest_cloud4_->header.stamp - latest_time).toSec()) > merge_timeout_)
        {
            return;
        }
        
        // Convert and merge point clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        merged_cloud->header.frame_id = target_frame_;
        
        // Process each sensor cloud
        addCloudToMerged(latest_cloud1_, merged_cloud);
        addCloudToMerged(latest_cloud2_, merged_cloud);
        addCloudToMerged(latest_cloud3_, merged_cloud);
        addCloudToMerged(latest_cloud4_, merged_cloud);
        
        // Publish merged cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*merged_cloud, output);
        output.header.stamp = latest_time;
        output.header.frame_id = target_frame_;
        pub_merged_cloud_.publish(output);
        
        ROS_DEBUG("Merged cloud published with %zu points", merged_cloud->size());
    }
    
    void addCloudToMerged(const sensor_msgs::PointCloud2::ConstPtr& msg,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& merged_cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *temp_cloud);
        
        if (use_transform_ && msg->header.frame_id != target_frame_)
        {
            // Transform point cloud to target frame
            try
            {
                tf::StampedTransform transform;
                tf_listener_.waitForTransform(target_frame_, msg->header.frame_id,
                                            msg->header.stamp, ros::Duration(0.1));
                tf_listener_.lookupTransform(target_frame_, msg->header.frame_id,
                                           msg->header.stamp, transform);
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl_ros::transformPointCloud(*temp_cloud, *transformed_cloud, transform);
                *merged_cloud += *transformed_cloud;
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN("Transform failed: %s", ex.what());
                // If transform fails, just add the cloud as-is
                *merged_cloud += *temp_cloud;
            }
        }
        else
        {
            // No transform needed
            *merged_cloud += *temp_cloud;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_merger");
    CloudMerger merger;
    ros::spin();
    return 0;
}
