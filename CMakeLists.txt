# 设定camke版本与项目名称
cmake_minimum_required(VERSION 3.26.4)
project(bundle_adjustment)

# 设定C++版本
set(CMAKE_CXX_STANDARD 17)

# 设定Debug模式
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/debug/bin/)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/debug/lib/)
    link_directories(${PROJECT_SOURCE_DIR}/build/debug/lib/)
    file(REMOVE_RECURSE ${EXECUTABLE_OUTPUT_PATH}/config/)
    file(COPY "config/" DESTINATION ${EXECUTABLE_OUTPUT_PATH}/config)
endif()
#设置release模式
if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/release/bin/)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/release/lib/)
    link_directories(${PROJECT_SOURCE_DIR}/build/release/lib/)
    file(REMOVE_RECURSE ${EXECUTABLE_OUTPUT_PATH}/config/)
    file(COPY "config/" DESTINATION ${EXECUTABLE_OUTPUT_PATH}/config)
endif()

# 寻找G2O
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3"
        "/usr/include/suitesparse")

# 寻找OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

# 链接为adjustment库
add_library(adjustment SHARED
            src/bundle_adjustment.cpp
            src/bundle_adjustment_param.cpp)
target_link_libraries(adjustment
                      g2o_core g2o_stuff g2o_cli
                      GL GLU cholmod
                      g2o_incremental g2o_interactive g2o_interface g2o_parser
                      g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg
                      g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d
                      ${OpenCV_LIBS}
                      ${QT_LIBRARIES}
                      ${QT_QTOPENGL_LIBRARY}
                      ${GLUT_LIBRARY}
                      ${OPENGL_LIBRARY})

# 配置可执行文件
add_executable(test_bundle_adjustment test/test_bundle_adjustment.cpp)
target_link_libraries(test_bundle_adjustment
                     adjustment)

# 配置测试样例
add_executable(test_bundle_adjustment_param test/test_bundle_adjustment_param.cpp)
target_link_libraries(test_bundle_adjustment_param
        adjustment)