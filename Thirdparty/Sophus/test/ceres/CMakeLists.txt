# Make sure Ceres knows where to find Eigen
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR})

# git clone https://ceres-solver.googlesource.com/ceres-solver
find_package(Ceres 1.6.0 QUIET)

# 设置 CMake 政策： 如果希望避免此类问题，可以显式设置 CMP0115 政策来定义 CMake 如何处理文件扩展名的缺失：
cmake_policy(SET CMP0115 NEW)
# 设置为 NEW 会要求显式指定文件扩展名，如果文件没有扩展名或路径不明确，会报错。

if(Ceres_FOUND)
    message(STATUS "CERES found")

    # Tests to run
    set(TEST_SOURCES test_ceres_se3)

    foreach(test_src ${TEST_SOURCES})

        # add_executable(${test_src} ${test_src}.cpp local_parameterization_se3) 
        # 在 3.10+ 版本中，CMake 会更加注重政策和语法的严格执行，并且在检测到潜在问题时，会直接抛出错误，而不仅仅是警告。所以上面语句是错误的！
        add_executable(${test_src} ${test_src}.cpp local_parameterization_se3.hpp)

        target_link_libraries(${test_src} sophus ${CERES_LIBRARIES})
        target_include_directories(${test_src} SYSTEM PRIVATE ${CERES_INCLUDE_DIRS})
        add_test(${test_src} ${test_src})
    endforeach(test_src)

endif(Ceres_FOUND)
