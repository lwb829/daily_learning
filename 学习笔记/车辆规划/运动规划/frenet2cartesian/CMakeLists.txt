cmake_minimum_required(VERSION 3.10.2)
project(frenet2cartesian)
# set(CMAKE_BUILD_TYPE "Debug")

find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}  # Eigen和Eigen3 基本可通用
  ${absl_INCLUDE_DIRS}
)


add_library(
    PATH_SEARCHER_LIB SHARED
    lib/binary.cpp
    lib/math_utils.cpp
    lib/vec2d.cpp
    lib/cartesian_frenet_conversion.cpp
)

add_executable(frenet2cartesian src/cartesian_frenet_conversion_test.cpp)
target_link_libraries(frenet2cartesian PATH_SEARCHER_LIB glog absl::strings)
