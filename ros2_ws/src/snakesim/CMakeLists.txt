cmake_minimum_required(VERSION 3.8)
project(snakesim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add include directories
include_directories(
  include
  ${SFML_INCLUDE_DIR}  # Include SFML headers
)

# Create executable
add_executable(snakesim_node src/snakesim_node.cpp src/Link.cpp)
ament_target_dependencies(snakesim_node ament_index_cpp)

# Link against necessary libraries
target_link_libraries(snakesim_node
  sfml-system
  sfml-window
  sfml-graphics
)

# Install executable
install(TARGETS
  snakesim_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY
  include/
  DESTINATION include)

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

ament_package()
