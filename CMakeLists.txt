cmake_minimum_required(VERSION 3.9)
project(TelenavSensorFusion)

set(CMAKE_CXX_STANDARD 11)

# set EIGEN_DIR variable to Eigen Library Path
# set(EIGEN_DIR "C:\\Eigendir\\Eigen")

# include the directory
# include_directories(${EIGEN_DIR})

add_executable(TelenavSensorFusion main.cpp tools.h tools.cpp Iteration.cpp
        Iteration.h kalman_filter.cpp kalman_filter.h measurement_package.h particle_filter.cpp particle_filter.h)

target_link_libraries(TelenavSensorFusion ${EIGEN_DIR})
