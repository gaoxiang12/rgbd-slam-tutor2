cmake_minimum_required( VERSION 2.8 )
project( rgbd-slam-tutor2 )

# 设置用debug还是release模式。debug允许断点，而release更快
#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_BUILD_TYPE Release )

# 设置编译选项
# 允许c++11标准、O3优化、多线程。match选项可避免一些cpu上的问题
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

# 常见依赖库：cv, eigen, pcl
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 二进制文件输出到bin
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# 库输出到lib
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

# 头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    )

# 源文件目录
add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/experiment/ )
