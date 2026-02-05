// NDT/VGICP-based Laser Odometry for A-LOAM
// This is an alternative backend that replaces point-to-line/point-to-plane ICP
// with NDT (Normal Distribution Transform) or VGICP (Voxelized GICP) registration
// 
// Corresponds to paper section 3.2.3: Voxel-based probabilistic registration
//
// Based on the original laserOdometry.cpp from A-LOAM but using NDT/VGICP for registration

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

// NDT registration
#include <pcl/registration/ndt.h>
// Optional: If fast_gicp is available, it can be used
// #include <fast_gicp/gicp/fast_vgicp.hpp>

constexpr double SCAN_PERIOD = 0.1;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// Store last transformation for incremental registration
Eigen::Matrix4d last_transformation = Eigen::Matrix4d::Identity();

// Odometry path
nav_msgs::Path laserPath;

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

// NDT parameters
double ndt_resolution = 1.0;  // Voxel grid resolution for NDT
double ndt_step_size = 0.1;   // Step size for More-Thuente line search
int ndt_max_iterations = 30;  // Maximum number of registration iterations
double ndt_transformation_epsilon = 0.01; // Transformation epsilon

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometryNDT");
    ros::NodeHandle nh;

    // Load parameters
    nh.param<double>("ndt_resolution", ndt_resolution, 1.0);
    nh.param<double>("ndt_step_size", ndt_step_size, 0.1);
    nh.param<int>("ndt_max_iterations", ndt_max_iterations, 30);
    nh.param<double>("ndt_transformation_epsilon", ndt_transformation_epsilon, 0.01);

    ROS_INFO("NDT-based Laser Odometry initialized");
    ROS_INFO("NDT parameters - resolution: %.2f, step_size: %.2f, max_iterations: %d",
             ndt_resolution, ndt_step_size, ndt_max_iterations);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    // Create NDT registration object
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setResolution(ndt_resolution);
    ndt.setStepSize(ndt_step_size);
    ndt.setMaximumIterations(ndt_max_iterations);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon);

    ros::Rate rate(100);
    int frameCount = 0;
    
    while (ros::ok())
    {
        ros::spinOnce();

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                ROS_WARN("Unsync messages! Removing oldest message.");
                mBuf.lock();
                // Remove the oldest message to allow resynchronization
                if (!cornerSharpBuf.empty()) cornerSharpBuf.pop();
                if (!cornerLessSharpBuf.empty()) cornerLessSharpBuf.pop();
                if (!surfFlatBuf.empty()) surfFlatBuf.pop();
                if (!surfLessFlatBuf.empty()) surfLessFlatBuf.pop();
                if (!fullPointsBuf.empty()) fullPointsBuf.pop();
                mBuf.unlock();
                continue;
            }

            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();

            TicToc t_whole;

            if (!systemInited)
            {
                systemInited = true;
                ROS_INFO("Initialization finished");
            }
            else
            {
                // NDT-based registration approach (Section 3.2.3)
                // Unlike original LOAM which uses point-to-line and point-to-plane correspondences,
                // NDT uses voxel-based probabilistic matching:
                // - Divides space into voxels
                // - Models each voxel as a Gaussian distribution (mean μ, covariance Σ)
                // - Optimizes transformation by maximizing likelihood
                // This approach is more robust for multi-sensor fusion and varying point densities
                
                // Combine corner and surface features for NDT registration
                pcl::PointCloud<PointType>::Ptr currentScan(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr lastScan(new pcl::PointCloud<PointType>());
                
                // Use all feature points for more robust NDT registration
                *currentScan += *cornerPointsLessSharp;
                *currentScan += *surfPointsLessFlat;
                
                *lastScan += *laserCloudCornerLast;
                *lastScan += *laserCloudSurfLast;

                if (lastScan->size() > 50 && currentScan->size() > 50)
                {
                    TicToc t_opt;

                    // Set input clouds for NDT
                    ndt.setInputSource(currentScan);
                    ndt.setInputTarget(lastScan);

                    // Use identity transformation as initial guess
                    // (NDT will find the transformation from lastScan to currentScan)
                    Eigen::Matrix4f initial_guess = last_transformation.cast<float>();

                    // Perform NDT registration
                    pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
                    ndt.align(*aligned, initial_guess);

                    if (ndt.hasConverged())
                    {
                        // Get the incremental transformation from last to current frame
                        Eigen::Matrix4f transformation = ndt.getFinalTransformation();
                        
                        // Store for next iteration
                        last_transformation = transformation.cast<double>();
                        
                        // Extract rotation and translation from incremental transform
                        Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
                        Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
                        
                        // Update accumulated pose
                        Eigen::Quaterniond q_incr(rotation.cast<double>());
                        Eigen::Vector3d t_incr = translation.cast<double>();
                        
                        // Accumulate: T_w_curr = T_w_last * T_last_curr
                        t_w_curr = q_w_curr * t_incr + t_w_curr;
                        q_w_curr = q_w_curr * q_incr;
                        q_w_curr.normalize();

                        ROS_DEBUG("NDT converged. Score: %.4f, Iterations: %d",
                                 ndt.getFitnessScore(), ndt.getFinalNumIteration());
                    }
                    else
                    {
                        ROS_WARN("NDT did not converge");
                    }

                    ROS_DEBUG("optimization costs: %f ms", t_opt.toc());
                }
                else
                {
                    ROS_WARN("Not enough points for NDT: last=%zu, current=%zu",
                            lastScan->size(), currentScan->size());
                }
            }

            // Update last frame
            *laserCloudCornerLast = *cornerPointsLessSharp;
            *laserCloudSurfLast = *surfPointsLessFlat;

            frameCount++;
            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                // Publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "/camera_init";
                laserOdometry.child_frame_id = "/laser_odom";
                laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsFlat);
                laserOdometry.pose.pose.orientation.x = q_w_curr.x();
                laserOdometry.pose.pose.orientation.y = q_w_curr.y();
                laserOdometry.pose.pose.orientation.z = q_w_curr.z();
                laserOdometry.pose.pose.orientation.w = q_w_curr.w();
                laserOdometry.pose.pose.position.x = t_w_curr.x();
                laserOdometry.pose.pose.position.y = t_w_curr.y();
                laserOdometry.pose.pose.position.z = t_w_curr.z();
                pubLaserOdometry.publish(laserOdometry);

                // Publish path
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserPath.header.stamp = laserOdometry.header.stamp;
                laserPath.poses.push_back(laserPose);
                laserPath.header.frame_id = "/camera_init";
                pubLaserPath.publish(laserPath);

                // Publish point clouds
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            ROS_DEBUG("whole laserOdometry time: %f ms", t_whole.toc());
        }

        rate.sleep();
    }
    return 0;
}
