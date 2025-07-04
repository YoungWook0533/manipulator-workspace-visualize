cmake_minimum_required(VERSION 3.8)
project(ws_visualize LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ▽ 컴파일러 옵션 ------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ▽ 의존 패키지 -------------------------------------------------------------
find_package(ament_cmake          REQUIRED)
find_package(eigen3_cmake_module  REQUIRED)   # Eigen3 찾기용 helper
find_package(Eigen3               REQUIRED NO_MODULE)
find_package(rclcpp               REQUIRED)
find_package(pinocchio            REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common)
find_package(sensor_msgs REQUIRED)
find_package(fcl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

# ▽ 실행 파일 ---------------------------------------------------------------
add_executable(ee_sampler_node src/ee_sampler_node.cpp)
add_executable(pcd_publisher_node src/pcd_publisher_node.cpp)

target_link_libraries(ee_sampler_node
  Eigen3::Eigen               
  ${FCL_LIBRARIES}
)
target_link_libraries(pcd_publisher_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ee_sampler_node
  rclcpp
  pinocchio
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  fcl
  visualization_msgs
)
ament_target_dependencies(pcd_publisher_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# ▽ 헤더 경로 (백업용 – 위 target_link_libraries로 충분하지만
#   배포 환경마다 다를 수 있으니 포함)
target_include_directories(ee_sampler_node SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

# ▽ 설치 -------------------------------------------------------------------
install(TARGETS ee_sampler_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS pcd_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# ▽ 테스트(lint) -----------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND    TRUE)
  set(ament_cmake_copyright_FOUND  TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
