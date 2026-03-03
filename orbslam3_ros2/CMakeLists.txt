cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orbslam3_msgs REQUIRED)

# ensure message include paths pulled in automatically


include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono  src/mono.cpp  src/slam_node.cc)
ament_target_dependencies(mono rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_executable(stereo  src/stereo_node.cpp  src/stereo.cpp  src/stereo_image_saver.cpp  src/slam_node.cc)
ament_target_dependencies(stereo rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_executable(stereo-inertial  src/stereo_inertial_node.cpp  src/stereo_inertial.cpp  src/stereo_image_saver.cpp  src/slam_node.cc)
ament_target_dependencies(stereo-inertial rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_executable(mono-inertial  src/mono-inertial.cpp  src/slam_node.cc)
ament_target_dependencies(mono-inertial rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_executable(mono-compressed  src/mono-compressed.cpp  src/slam_node.cc)
ament_target_dependencies(mono-compressed rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_executable(mono-inertial-compressed  src/mono-inertial-compressed.cpp  src/slam_node.cc)
ament_target_dependencies(mono-inertial-compressed rclcpp rclcpp_components sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV tf2_geometry_msgs tf2_ros nav_msgs std_srvs orbslam3_msgs)

add_library(stereo_slam_component SHARED src/stereo.cpp src/slam_node.cc)

# Dependências ROS 2
ament_target_dependencies(stereo_slam_component 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  cv_bridge 
  message_filters 
  tf2_geometry_msgs 
  tf2_ros 
  nav_msgs 
  std_srvs
  orbslam3_msgs
  ORB_SLAM3 Pangolin OpenCV
)

# Linkagem de bibliotecas externas (CRÍTICO: Sem isso dá erro de "undefined symbol" ao rodar)
target_link_libraries(stereo_slam_component 
  ${ORB_SLAM3_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
)

# Registro do Plugin
rclcpp_components_register_node(stereo_slam_component 
  PLUGIN "orbslam3_ros2::StereoSlamNode" 
  EXECUTABLE stereo_slam_node
)
# Stereo Inertial SLAM Component
add_library(stereo_inertial_slam_component SHARED src/stereo_inertial.cpp src/slam_node.cc)

ament_target_dependencies(stereo_inertial_slam_component 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  cv_bridge 
  message_filters
  tf2_geometry_msgs 
  tf2_ros 
  nav_msgs 
  std_srvs
  orbslam3_msgs
  ORB_SLAM3 Pangolin OpenCV
)

# Linkagem de bibliotecas externas (CRÍTICO: Sem isso dá erro de "undefined symbol" ao rodar)
target_link_libraries(stereo_inertial_slam_component 
  ${ORB_SLAM3_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
)

# Registro do Plugin
rclcpp_components_register_node(stereo_inertial_slam_component 
  PLUGIN "orbslam3_ros2::StereoInertialSlamNode" 
  EXECUTABLE stereo_inertial_slam_node
)


# 2. Componente do Saver (Gravação de Imagens)
add_library(stereo_saver_component SHARED src/stereo_image_saver.cpp)

ament_target_dependencies(stereo_saver_component 
  rclcpp 
  rclcpp_components 
  cv_bridge 
  sensor_msgs 
  message_filters
  ORB_SLAM3 Pangolin OpenCV
)

target_link_libraries(stereo_saver_component 
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(stereo_saver_component 
  PLUGIN "orbslam3_ros2::StereoImageSaverNode" 
  EXECUTABLE stereo_saver_node
)

# ==============================================================================
# INSTALAÇÃO
# ==============================================================================

# 1. Instalar as bibliotecas dos componentes (ESSENCIAL para o Launch achar)
install(TARGETS 
  stereo_slam_component 
  stereo_inertial_slam_component
  stereo_saver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mono mono-inertial stereo stereo-inertial mono-compressed mono-inertial-compressed DESTINATION lib/${PROJECT_NAME})
# Install launch files.
install(DIRECTORY launch config vocabulary
 DESTINATION share/${PROJECT_NAME}/)

ament_package()

