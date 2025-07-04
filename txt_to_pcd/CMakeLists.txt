# Set project's name
PROJECT(PointCloudSegmentation)

###############################################################################
# CMake settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

# Enable C++14 standard
add_definitions(-std=c++14)

# OpenMP
FIND_PACKAGE(OpenMP REQUIRED)
IF(OPENMP_FOUND)
    OPTION(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
ENDIF(OPENMP_FOUND)

IF(OPENMP_FOUND AND WITH_OPENMP)
    MESSAGE(STATUS "With OpenMP")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
ELSE(OPENMP_FOUND AND WITH_OPENMP)
    MESSAGE(STATUS "Without OpenMP")
ENDIF(OPENMP_FOUND AND WITH_OPENMP)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# Point Cloud Library (PCL)
FIND_PACKAGE(PCL 1.10 REQUIRED)

# Add PCL and OpenCV definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

# Include directories for PCL and OpenCV
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Link directories for PCL and OpenCV
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# Add executable and link it with PCL and OpenCV libraries
add_executable(txt2pcd src/txt2pcd.cpp)
target_link_libraries(txt2pcd ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(merge_pcd src/merge_pcd.cpp)
target_link_libraries(merge_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS})
