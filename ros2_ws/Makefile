
# Set the ROS2 workspace directory
ROS2_WS_DIR := /home/hgtllys/Documents/Projects/densoVP6242/ros2_ws

# Set the name of the ROS2 package
PACKAGE_NAME := snakesim

# Executable name
EXECUTABLE_NAME := snakesim_node

# Set the source directory of the ROS2 package
PACKAGE_SRC_DIR := $(ROS2_WS_DIR)/src/$(PACKAGE_NAME)

# Set the build directory of the ROS2 package
PACKAGE_BUILD_DIR := $(ROS2_WS_DIR)/build/$(PACKAGE_NAME)

# Set the install directory of the ROS2 package
PACKAGE_INSTALL_DIR := $(ROS2_WS_DIR)/install/$(PACKAGE_NAME)

# Build the ROS2 package
build:
	@echo "Building $(PACKAGE_NAME)..."
	@colcon build --symlink-install --packages-select $(PACKAGE_NAME)

# Run the ROS2 package
run:
	@echo "Running $(PACKAGE_NAME)..."
	ros2 run $(PACKAGE_NAME) $(EXECUTABLE_NAME)

# Clean the ROS2 package
clean:
	@echo "Cleaning $(PACKAGE_NAME)..."
	@rm -rf $(PACKAGE_BUILD_DIR) $(PACKAGE_INSTALL_DIR)

.PHONY: build run clean
