cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_gazebo)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros  
  rospy
)

# Declare a catkin package
catkin_package(
  # INCLUDE_DIRS include  # Uncomment if you have include files
  # LIBRARIES my_rb1_gazebo  # Uncomment if you create a library
  CATKIN_DEPENDS gazebo_ros rospy  # Corrected from 'gazebo' to 'gazebo_ros'
  # DEPENDS system_lib  # Uncomment and replace with actual system dependencies, if any
)

# Include directories
include_directories(
  # include  # Uncomment if you have include files
  ${catkin_INCLUDE_DIRS}
)

# Here you would add declarations for any libraries or executables, if you have them.
# For example, if you're creating a library:
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_rb1_gazebo.cpp
# )

# If creating an executable:
# add_executable(${PROJECT_NAME}_node src/my_rb1_gazebo_node.cpp)

# And to link the libraries:
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# If you have additional tasks like installing scripts, libraries, etc., add them below.
