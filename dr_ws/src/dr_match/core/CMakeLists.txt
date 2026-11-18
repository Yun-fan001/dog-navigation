# dr_match core —— C++ 无 ROS 依赖纯库（计划书 A10–A12）
#
# 本目录在 Ubuntu 20.04 环境构建（Windows 本机无 C++ 工具链）。
# 接口已冻结于 include/dr_match/matcher.h，与 Python 参考实现
# py/dr_match/ 一一对应（golden reference 交叉验证）。
#
# 构建（Ubuntu）：
#   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
#   ctest --test-dir build          # GoogleTest：验收标准即断言
cmake_minimum_required(VERSION 3.10)
project(dr_match_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 纯库：无 ROS 依赖（计划书 §6.3 纪律）
find_package(Eigen3 3.3 REQUIRED)

add_library(dr_match_core
  src/graph.cpp
  src/matcher.cpp
  src/geometry.cpp
)
target_include_directories(dr_match_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(dr_match_core PUBLIC Eigen3::Eigen)

# 单测（Ubuntu 安装 libgtest-dev 后启用）
include(CTest)
if(BUILD_TESTING)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(test_matcher test/test_matcher.cpp)
    target_link_libraries(test_matcher dr_match_core GTest::GTest GTest::Main)
    add_test(NAME matcher COMMAND test_matcher)
  endif()
endif()
