cmake_minimum_required(VERSION 3.0.0)
project(pid_test)
set(Sophus_DIR "/home/xxx/Sophus/build")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
aux_source_directory(./src ALL_SRCS)
include_directories(include
${OpenCV_INCLUDE_DIRS}
${EIGEN_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${fftw_INCLUDE_DIRS})

set(SRC ./cc)

add_executable(pid_test main.cpp ${ALL_SRCS})
target_link_libraries(pid_test ${OpenCV_LIBS} ${CERES_LIBRARIES} fftw3)