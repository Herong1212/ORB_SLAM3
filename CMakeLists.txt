cmake_minimum_required(VERSION 3.20) # 原本是 3.4，最高到 3.17 了，再新就不兼容了！
project(ORB_SLAM3)

# 从 CMake 3.12 开始，处理CUDA和CUDA架构的方式发生了变化，特别是当使用find_package(Torch)时，CMake会要求指定CUDA_ARCHITECTURES，以确保编译能够生成针对你硬件的代码。
# 设置 CUDA 架构    【NVIDIA RTX 3060 使用的是 Ampere 架构(主要用于RTX 30系列显卡,使用 ./deviceQuery 查询），计算能力（compute capability）为 sm_86。】
# set(CUDA_ARCHITECTURES "86") 

# 设置编译选项 Release
set(CMAKE_BUILD_TYPE Release) # 在这儿修改为 Debug 好像不太管用
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
message("")

# 新增定义👇
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     add_definitions(-DREGISTER_TIMES)
# endif()

# C 编译选项（把 -Wall 删了）
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# C++ 编译选项（把 -Wall 删了）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# C 和 C++ 的发布模式编译选项，进一步优化本地性能
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# CMake 会自动检测编译器是否支持 C++11 或者 C++0x（这是 C++11 的一个早期版本标志，有些旧编译器使用它来表示 C++11）
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX17)
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
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

add_definitions(-w)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message("")

# set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/libtorch)
set(CMAKE_PREFIX_PATH "/usr/local/lib/libtorch")
find_package(Torch REQUIRED)
message("")
message("Torch VERSION:" ${Torch_VERSION})
message(STATUS "Torch include dirs: " ${TORCH_INCLUDE_DIRS})
message(STATUS "Torch library dirs: " ${TORCH_LIBRARIES})
message("")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
message("")

find_package(PCL REQUIRED)
message(STATUS "Pcl: " ${PCL_VERSION})
message("")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3: " ${Eigen3_VERSION})
message("")

find_package(Pangolin REQUIRED)
message(STATUS "Pangolin: " ${Pangolin_VERSION})
message("")

find_package(realsense2)
message(STATUS "realsense2: " ${realsense2_VERSION})
message("")

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}

)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
message(STATUS "CMAKE_LIBRARY_OUTPUT_DIRECTORY: " ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
message("")

add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Viewer.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
    src/PointCloudMapper.cpp
    src/YoloDetect.cpp
    include/System.h
    include/Tracking.h
    include/LocalMapping.h
    include/LoopClosing.h
    include/ORBextractor.h
    include/ORBmatcher.h
    include/FrameDrawer.h
    include/Converter.h
    include/MapPoint.h
    include/KeyFrame.h
    include/Atlas.h
    include/Map.h
    include/MapDrawer.h
    include/Optimizer.h
    include/Frame.h
    include/KeyFrameDatabase.h
    include/Sim3Solver.h
    include/Viewer.h
    include/ImuTypes.h
    include/G2oTypes.h
    include/CameraModels/GeometricCamera.h
    include/CameraModels/Pinhole.h
    include/CameraModels/KannalaBrandt8.h
    include/OptimizableTypes.h
    include/MLPnPsolver.h
    include/GeometricTools.h
    include/TwoViewReconstruction.h
    include/SerializationUtils.h
    include/Config.h
    include/Settings.h
    include/PointCloudMapper.h
    include/YoloDetect.h)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
    ${PCL_LIBRARIES}
    ${TORCH_LIBRARIES}
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
        ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
        ${realsense2_LIBRARY}
    )
endif()


#---------------------------------------------------------------------------------- Build examples ------------------------------------------------------------------------------------------------

# # 1、RGB-D examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(rgbd_realsense_D435i Examples/RGB-D/rgbd_realsense_D435i.cc)
#     target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
# endif()


# # 2、RGB-D inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

# if(realsense2_FOUND)
#     add_executable(rgbd_inertial_realsense_D435i Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
#     target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# # 3、Stereo examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})

# add_executable(stereo_tum_vi Examples/Stereo/stereo_tum_vi.cc)
# target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_realsense_t265 Examples/Stereo/stereo_realsense_t265.cc)
#     target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

#     add_executable(stereo_realsense_D435i Examples/Stereo/stereo_realsense_D435i.cc)
#     target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
# endif()

# # 4、Stereo Inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

# add_executable(stereo_inertial_euroc Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
# target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

# add_executable(stereo_inertial_tum_vi Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
# target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(stereo_inertial_realsense_t265 Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#     target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

#     add_executable(stereo_inertial_realsense_D435i Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#     target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
# endif()

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
# if(realsense2_FOUND)
#     add_executable(recorder_realsense_D435i Examples/Calibration/recorder_realsense_D435i.cc)
#     target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

#     add_executable(recorder_realsense_T265 Examples/Calibration/recorder_realsense_T265.cc)
#     target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
# endif()

# # 5、Monocular examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

# add_executable(mono_tum_vi Examples/Monocular/mono_tum_vi.cc)
# target_link_libraries(mono_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_realsense_t265 Examples/Monocular/mono_realsense_t265.cc)
#     target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

#     add_executable(mono_realsense_D435i Examples/Monocular/mono_realsense_D435i.cc)
#     target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
# endif()

# # 6、Monocular inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

# add_executable(mono_inertial_euroc Examples/Monocular-Inertial/mono_inertial_euroc.cc)
# target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# add_executable(mono_inertial_tum_vi Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
# target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# if(realsense2_FOUND)
#     add_executable(mono_inertial_realsense_t265 Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
#     target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

#     add_executable(mono_inertial_realsense_D435i Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
#     target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
# endif()


