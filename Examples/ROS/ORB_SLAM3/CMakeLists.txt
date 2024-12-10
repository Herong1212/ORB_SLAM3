cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros)

message("")
message("----------------------初始化完成！-----------------------------------" \n)

set(ROS_BUILD_TYPE Release)
# set(ROS_BUILD_TYPE Debug)
message("ROS_BUILD_TYPE: " ${ROS_BUILD_TYPE})
message("")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
message("")

# CMake 会自动检测编译器是否支持 C++11 或者 C++0x（这是 C++11 的一个早期版本标志，有些旧编译器使用它来表示 C++11）
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
    message("")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
message("")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../../../Thirdparty/libtorch)

# 2.5.1
set(CMAKE_PREFIX_PATH  "/opt/ros/noetic" ${CMAKE_PREFIX_PATH})
find_package(Torch REQUIRED)
message("")
message("Torch VERSION:" ${Torch_VERSION})
message(STATUS "Torch include dirs: " ${TORCH_INCLUDE_DIRS})
message(STATUS "Torch library dirs: " ${TORCH_LIBRARIES})
message("")

# 查找 catkin，并引入所需的组件（例如 roscpp 和 std_msgs）
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    tf
    geometry_msgs
    # 其他依赖项
)

# 4.2.0
find_package(OpenCV REQUIRED)
message("")
message("OpenCV Version: " ${OpenCV_VERSION})
message("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
message("OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES})
message("")

# 3.3.7
find_package(Eigen3 REQUIRED)
message("Eigen3 version: ${Eigen3_VERSION}")
message("")

# 0.6
find_package(Pangolin REQUIRED)
message("Pangolin version: ${Pangolin_VERSION}")
message("")

# 1.0.0
find_package(PCL REQUIRED)

# /home/dsg/orbslam3_ghr/src/ORB_SLAM3/Examples/ROS/ORB_SLAM3
message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../../../
    ${PROJECT_SOURCE_DIR}/../../../include
    ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}

)
message("")

set(LIBS
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
    -lboost_system
    ${catkin_LIBRARIES}
)
message("")

# 声明catkin的包
# catkin_package(
#     # 如果有库或其他需要导出的内容可以在这里填写
#     CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
# )

# ------------------------------------------- 👇编译可执行文件👇 ----------------------------------------------------

# 更改可执行文件的输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# Node for monocular camera
add_executable(Mono src/ros_mono.cc) # 单目
target_link_libraries(Mono ${LIBS})

# Node for stereo camera
add_executable(Stereo src/ros_stereo.cc) # 双目
target_link_libraries(Stereo ${LIBS})

# Node for RGB-D camera
add_executable(RGBD src/ros_rgbd.cc) # RGB-D
target_link_libraries(RGBD ${LIBS})

# Node for monocular-inertial camera
add_executable(Mono_Inertial src/ros_mono_inertial.cc) # 单目 + IMU
target_link_libraries(Mono_Inertial ${LIBS})

# Node for stereo-inertial camera
add_executable(Stereo_Inertial src/ros_stereo_inertial.cc) # 双目 + IMU
target_link_libraries(Stereo_Inertial ${LIBS})

# new：Node for rgbd-inertial camera
add_executable(RGBD_Inertial src/ros_rgbd_inertial.cc) # RGB-D + IMU
target_link_libraries(RGBD_Inertial ${LIBS})


