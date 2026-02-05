# NDT/VGICP Integration for A-LOAM

## Overview

This implementation adds NDT (Normal Distribution Transform) based registration to A-LOAM, corresponding to the methodology described in paper section 3.2.3. This provides an alternative to the traditional point-to-line and point-to-plane ICP approach.

## New Features

### 1. Multi-Sensor Point Cloud Fusion (`cloudMerger.cpp`)

A ROS node that merges point clouds from multiple sensors (designed for 4 small FOV sensors) into a single unified point cloud.

**Features:**
- Subscribes to 4 sensor topics
- Transforms all point clouds to a common reference frame (`base_link`)
- Synchronizes and merges point clouds based on timestamps
- Publishes unified point cloud for SLAM processing

**Parameters:**
- `target_frame` (string, default: "base_link"): Target coordinate frame for merged cloud
- `use_transform` (bool, default: true): Enable TF transformation
- `merge_timeout` (double, default: 0.1): Maximum time difference for synchronization (seconds)

### 2. NDT-based Laser Odometry (`laserOdometryNDT.cpp`)

An alternative odometry node that uses NDT for point cloud registration instead of the traditional LOAM ICP approach.

**Key Differences from Original LOAM:**
- Uses voxel-based Normal Distribution Transform (NDT) for registration
- Corresponds to paper section 3.2.3: voxel grid subdivision and probabilistic density functions
- More robust to noise and unstructured environments
- Better suited for multi-sensor fusion scenarios

**Parameters:**
- `ndt_resolution` (double, default: 1.0): Voxel grid resolution in meters
- `ndt_step_size` (double, default: 0.1): Step size for More-Thuente line search
- `ndt_max_iterations` (int, default: 30): Maximum number of registration iterations
- `ndt_transformation_epsilon` (double, default: 0.01): Transformation epsilon for convergence

## Architecture

### Traditional A-LOAM Pipeline:
```
Sensor → Feature Extraction → Point-to-Line/Plane ICP → Mapping
```

### NDT-Enhanced A-LOAM Pipeline:
```
4 Sensors → Cloud Merger → Feature Extraction → NDT Registration → Mapping
```

## Correspondence to Paper Sections

### Section 3.2.2 - Feature Extraction
- **Implementation**: `scanRegistration.cpp` (unchanged from original A-LOAM)
- **Method**: Curvature-based feature extraction (corner points and surface points)
- **Formula**: Smoothness calculation based on neighboring points

### Section 3.2.3 - Voxel-based Probabilistic Registration
- **Implementation**: `laserOdometryNDT.cpp` (new)
- **Method**: NDT (Normal Distribution Transform)
- **Key Concepts**:
  - Voxel grid subdivision
  - Gaussian distribution modeling (mean μ and covariance Σ)
  - Maximum likelihood estimation
  - Newton's method for optimization

## Usage

### For Standard Single Sensor (Original LOAM):
```bash
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

### For Multi-Sensor with NDT Backend:
```bash
roslaunch aloam_velodyne aloam_velodyne_ndt.launch
```

### Configuration for Your Sensors:

Edit `launch/aloam_velodyne_ndt.launch` and update the sensor topic remappings:

```xml
<remap from="/sensor1/points" to="/your_sensor1_topic"/>
<remap from="/sensor2/points" to="/your_sensor2_topic"/>
<remap from="/sensor3/points" to="/your_sensor3_topic"/>
<remap from="/sensor4/points" to="/your_sensor4_topic"/>
```

## Parameter Tuning

### NDT Resolution
- **Smaller values (0.5-1.0m)**: Better accuracy, higher computational cost
- **Larger values (1.0-2.0m)**: Faster processing, may lose fine details
- **Recommended**: Start with 1.0m and adjust based on environment size

### NDT Max Iterations
- **Lower values (10-20)**: Faster but may not converge
- **Higher values (30-50)**: More likely to converge but slower
- **Recommended**: 30 for most cases

## Implementation Notes

### Why NDT for Multi-Sensor Fusion?
1. **Robustness**: NDT is more robust to point density variations, which is common when merging multiple sensors
2. **Efficiency**: Voxel-based approach handles large point clouds efficiently
3. **Probabilistic**: Better handling of measurement uncertainty from multiple sensors
4. **Flexibility**: Works well with both structured (scan-line) and unstructured point clouds

### Comparison: ICP vs NDT

| Aspect | Traditional ICP (LOAM) | NDT (This Implementation) |
|--------|------------------------|---------------------------|
| Point Matching | Explicit correspondences | Probabilistic (no explicit matches) |
| Feature Requirements | Needs distinct features | Works with all points |
| Computational Cost | O(n log n) with KD-tree | O(n) with voxel lookup |
| Robustness to Noise | Moderate | High |
| Best Use Case | Structured environments | Unstructured/multi-sensor |

## Extending to VGICP

For even better performance, you can replace the standard PCL NDT with Fast-GICP library:

1. Install fast_gicp:
```bash
cd ~/catkin_ws/src
git clone https://github.com/SMRT-AIST/fast_gicp
cd ~/catkin_ws
catkin_make
```

2. Update `laserOdometryNDT.cpp` to use VGICP:
```cpp
#include <fast_gicp/gicp/fast_vgicp.hpp>

// Replace pcl::NormalDistributionsTransform with:
fast_gicp::FastVGICP<PointType, PointType> vgicp;
vgicp.setResolution(ndt_resolution);
vgicp.setNumThreads(4);
```

## Simulation Testing

For Gazebo simulation with 4 small FOV sensors:

1. Define sensor transforms in your URDF/xacro
2. Launch Gazebo with your robot model
3. Verify TF tree has transforms from `base_link` to each sensor frame
4. Launch the NDT-based A-LOAM:
```bash
roslaunch aloam_velodyne aloam_velodyne_ndt.launch
```

## References

- **Original LOAM**: J. Zhang and S. Singh. "LOAM: Lidar Odometry and Mapping in Real-time"
- **A-LOAM**: Advanced implementation by HKUST Aerial Robotics
- **NDT**: P. Biber and W. Straßer. "The normal distributions transform: A new approach to laser scan matching"
- **VGICP**: Kenji Koide et al. "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration"

## Troubleshooting

### NDT Not Converging
- Increase `ndt_max_iterations`
- Decrease `ndt_resolution` for finer voxels
- Check if initial guess from IMU/odometry is available

### Cloud Merger Not Publishing
- Verify all 4 sensor topics are publishing
- Check TF tree has transforms for all sensor frames
- Adjust `merge_timeout` if sensors have different rates

### Build Errors
- Ensure PCL version >= 1.7
- Install `pcl_ros`: `sudo apt-get install ros-$ROS_DISTRO-pcl-ros`
- Check all dependencies in package.xml are installed

## License

Same as original A-LOAM (BSD License)
