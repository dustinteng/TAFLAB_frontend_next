# Use the official ROS2 Humble image
FROM ros:humble-ros-base

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rclpy \
    ros-humble-std-msgs

# Set the ROS environment
ENV ROS_DISTRO=humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a workspace and copy the package into it
RUN mkdir -p /ros2_ws/src
COPY src /ros2_ws/src/

# Verify that the package is copied correctly
RUN ls /ros2_ws/src/boat_location_publisher

# Build the workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Set the default command to run your ROS2 node
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run boat_location_publisher fake_boat_location"]
