
cmake_minimum_required(VERSION 3.10)
project(NavSense_ORB_SLAM3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ORB_SLAM3_DIR "$ENV{HOME}/opt/orbslam-deps/ORB_SLAM3")

find_package(OpenCV 4.6 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(test_sample1_rgbd
    test_sample1_rgbd.cpp
)

target_include_directories(test_sample1_rgbd PRIVATE
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(test_sample1_rgbd PRIVATE
    Eigen3::Eigen
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    boost_serialization
    crypto
)
