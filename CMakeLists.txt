cmake_minimum_required( VERSION 2.8 )
project( rgbd-slam-tutor2 )

set( CMAKE_BUILD_TYPE Release )

set( CMAKE_CXX_COMPILER "clang++" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL 1.7 REQUIRED )
find_package( G2O )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( Cholmod )
include_directories( ${CHOLMOD_INCLUDE_DIR} )


set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/Thirdparty/
    ${EIGEN3_INCLUDE_DIR}
    )

add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/experiment/ )
