# Use the official ROS 2 Humble base image for ARM64 architecture
FROM arm64v8/ros:humble-ros-base

# Set environment variables to ensure non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-dev \
    libffi-dev \
    libssl-dev \
    can-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install python-can library for CAN communication (if needed)
RUN pip3 install python-can

# Install ROS 2 packages like rosbridge-server (if needed)
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rosbridge-server \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create the ROS2 workspace and set it as the working directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Copy the ROS2 packages from your Mac to the Docker container
COPY src/boat_location_publisher /ros2_ws/src/boat_location_publisher

# Build the ROS2 workspace with colcon
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Ensure the workspace is sourced upon container startup
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Expose any necessary ports (if needed by your ROS2 app)
EXPOSE 3333

# Set the default command to run your ROS2 node
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run boat_location_publisher fake_boat_location"]
