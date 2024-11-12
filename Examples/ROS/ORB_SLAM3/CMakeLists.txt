cmake_minimum_required(VERSION 3.20)
project(orb_slam3_ros)

# 用于 在 ROS 项目中引入 rosbuild 编译系统，rosbuild 是 ROS 中的一个较旧的构建工具，通常用于 ROS 早期版本（如 ROS 1 的旧发行版）。
# rosbuild 已被废弃：在新版本的 ROS（如 ROS Noetic、ROS 2）中，推荐使用 catkin 或 ament 而非 rosbuild。也就是在本机上不需要该行代码！
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# $ENV{ROS_ROOT} 表示访问环境变量 ROS_ROOT，该变量通常指向 ROS 的安装目录，比如 /opt/ros/<distro>。
# message("ROS_ROOT: " ${ROS_ROOT})

# rosbuild_init() 会初始化 rosbuild 的一些默认设置，加载与构建、依赖关系和文件路径相关的配置。
# 它会在 CMakeLists.txt 文件中设置编译选项、包含路径和链接库等，以便后续可以使用 rosbuild 提供的构建功能。
# 在 ROS Noetic 中，因为 rosbuild 已不再推荐使用，建议用 catkin 来代替 rosbuild_init() 的功能。
# rosbuild_init()

message("")
message("----------------------初始化完成！-----------------------------------" \n)

set(ROS_BUILD_TYPE Release)
# set(ROS_BUILD_TYPE Debug)

message("ROS_BUILD_TYPE: " ${ROS_BUILD_TYPE})
message("")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
message("")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
message("")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

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

# 要保证整个工程的 opencv 版本一致，包括 dbow，源码以及 ros 相关的
# 3 4 都可以正常运行
find_package(OpenCV REQUIRED)
message("OpenCV Version: " ${OpenCV_VERSION})
message("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
message("OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES})
message("")

find_package(Eigen3 REQUIRED)
message("Eigen3 version: ${Eigen3_VERSION}")
message("")
find_package(Pangolin REQUIRED)
message("Pangolin version: ${Pangolin_VERSION}")
message("")

message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR}) # /home/dsg/orbslam3_ghr/src/ORB_SLAM3/Examples/ROS/ORB_SLAM3
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../../../
    ${PROJECT_SOURCE_DIR}/../../../include
    ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
    ${Pangolin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
message("标记第一处地点！")
message("")

set(LIBS
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
    -lboost_system
    ${catkin_LIBRARIES}
)
message("标记第二处地点！")
message("")

# 声明catkin的包
catkin_package(
    # 如果有库或其他需要导出的内容可以在这里填写
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

# ------------------------------------------- 👇编译可执行文件👇 ----------------------------------------------------
# 如果采用 rosbuild 方法，那就用这个。如果使用 catkin 方法，直接使用标准的 CMake  编译命令即可。
# rosbuild_add_executable(Mono src/ros_mono.cc)     # 单目
# target_link_libraries(Mono ${LIBS})

# 更改可执行文件的输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src)

# Node for monocular camera
add_executable(Mono src/ros_mono.cc) # 单目
target_link_libraries(Mono ${LIBS})

# set_target_properties(Mono PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin333)

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

# Node for rgbd-inertial camera
add_executable(RGBD_Inertial src/ros_rgbd_inertial.cc) # RGB-D + IMU
target_link_libraries(RGBD_Inertial ${LIBS})


