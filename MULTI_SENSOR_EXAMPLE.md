# Example Multi-Sensor Configuration for Gazebo

This document provides an example configuration for testing the multi-sensor NDT-based A-LOAM in a Gazebo simulation environment.

## URDF/Xacro Example for 4 Small FOV Sensors

Here's an example of how to configure 4 sensors positioned around a robot:

```xml
<?xml version="1.0"?>
<robot name="multi_sensor_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Sensor 1: Front -->
  <link name="sensor1_link"/>
  <joint name="sensor1_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor1_link"/>
    <origin xyz="0.25 0.0 0.1" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="sensor1_link">
    <sensor type="ray" name="sensor1">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-0.52</min_angle>  <!-- 30 degrees FOV -->
            <max_angle>0.52</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.26</min_angle>
            <max_angle>0.26</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.3</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_controller1" filename="libgazebo_ros_velodyne_laser.so">
        <topicName>/sensor1/velodyne_points</topicName>
        <frameName>sensor1_link</frameName>
        <organize_cloud>false</organize_cloud>
        <min_range>0.3</min_range>
        <max_range>100.0</max_range>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Sensor 2: Right -->
  <link name="sensor2_link"/>
  <joint name="sensor2_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor2_link"/>
    <origin xyz="0.0 -0.25 0.1" rpy="0 0 -1.5708"/>  <!-- -90 degrees -->
  </joint>
  
  <gazebo reference="sensor2_link">
    <sensor type="ray" name="sensor2">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-0.52</min_angle>
            <max_angle>0.52</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.26</min_angle>
            <max_angle>0.26</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.3</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_controller2" filename="libgazebo_ros_velodyne_laser.so">
        <topicName>/sensor2/velodyne_points</topicName>
        <frameName>sensor2_link</frameName>
        <organize_cloud>false</organize_cloud>
        <min_range>0.3</min_range>
        <max_range>100.0</max_range>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Sensor 3: Back -->
  <link name="sensor3_link"/>
  <joint name="sensor3_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor3_link"/>
    <origin xyz="-0.25 0.0 0.1" rpy="0 0 3.14159"/>  <!-- 180 degrees -->
  </joint>
  
  <gazebo reference="sensor3_link">
    <sensor type="ray" name="sensor3">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-0.52</min_angle>
            <max_angle>0.52</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.26</min_angle>
            <max_angle>0.26</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.3</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_controller3" filename="libgazebo_ros_velodyne_laser.so">
        <topicName>/sensor3/velodyne_points</topicName>
        <frameName>sensor3_link</frameName>
        <organize_cloud>false</organize_cloud>
        <min_range>0.3</min_range>
        <max_range>100.0</max_range>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Sensor 4: Left -->
  <link name="sensor4_link"/>
  <joint name="sensor4_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor4_link"/>
    <origin xyz="0.0 0.25 0.1" rpy="0 0 1.5708"/>  <!-- 90 degrees -->
  </joint>
  
  <gazebo reference="sensor4_link">
    <sensor type="ray" name="sensor4">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-0.52</min_angle>
            <max_angle>0.52</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.26</min_angle>
            <max_angle>0.26</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.3</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_controller4" filename="libgazebo_ros_velodyne_laser.so">
        <topicName>/sensor4/velodyne_points</topicName>
        <frameName>sensor4_link</frameName>
        <organize_cloud>false</organize_cloud>
        <min_range>0.3</min_range>
        <max_range>100.0</max_range>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Launch Sequence

1. **Start Gazebo with your robot:**
```bash
roslaunch your_robot_description gazebo.launch
```

2. **Verify sensor topics are publishing:**
```bash
rostopic list | grep velodyne_points
# Should show:
# /sensor1/velodyne_points
# /sensor2/velodyne_points
# /sensor3/velodyne_points
# /sensor4/velodyne_points
```

3. **Check TF tree:**
```bash
rosrun tf view_frames
# Verify transforms exist:
# base_link -> sensor1_link
# base_link -> sensor2_link
# base_link -> sensor3_link
# base_link -> sensor4_link
```

4. **Launch NDT-based A-LOAM:**
```bash
roslaunch aloam_velodyne aloam_velodyne_ndt.launch
```

## Visualizing in RViz

Add the following displays in RViz:
- **PointCloud2** for `/merged_cloud` (merged sensor data)
- **PointCloud2** for `/laser_cloud_sharp` (corner features)
- **PointCloud2** for `/laser_cloud_flat` (surface features)
- **Odometry** for `/laser_odom_to_init` (estimated trajectory)
- **Path** for `/laser_odom_path` (trajectory visualization)
- **TF** to see sensor frames

## Testing Without Gazebo

If you want to test with bag files instead of Gazebo:

1. **Record test data from 4 sensors:**
```bash
rosbag record /sensor1/velodyne_points /sensor2/velodyne_points \
               /sensor3/velodyne_points /sensor4/velodyne_points \
               /tf /tf_static -O test_4_sensors.bag
```

2. **Play back and test:**
```bash
# Terminal 1: Launch A-LOAM
roslaunch aloam_velodyne aloam_velodyne_ndt.launch

# Terminal 2: Play bag file
rosbag play test_4_sensors.bag
```

## Troubleshooting

### Sensors not synchronized
If you see "Unsync messages!" warnings, adjust the `merge_timeout` parameter:
```xml
<node pkg="aloam_velodyne" type="cloudMerger" name="cloudMerger">
    <param name="merge_timeout" type="double" value="0.2"/>  <!-- Increase if needed -->
</node>
```

### Transform errors
If cloudMerger reports "Transform failed", check:
```bash
# See if transforms are being published
rosrun tf tf_echo base_link sensor1_link

# If not, make sure your URDF is loaded:
rosrun robot_state_publisher robot_state_publisher
```

### NDT not converging
If you see many "NDT did not converge" warnings:
- Reduce `ndt_resolution` (e.g., from 1.0 to 0.5)
- Increase `ndt_max_iterations` (e.g., from 30 to 50)
- Check if merged cloud has enough points: `rostopic echo /merged_cloud --noarr`

## Performance Tuning

For better performance in simulation:
- Reduce point cloud density in Gazebo sensor configuration (`<samples>` tags)
- Increase NDT resolution for faster processing: `ndt_resolution: 1.5`
- Use fewer feature points in scanRegistration by adjusting parameters

For better accuracy:
- Decrease NDT resolution: `ndt_resolution: 0.5`
- Increase samples in Gazebo sensors
- Use smaller `merge_timeout` for tighter synchronization
