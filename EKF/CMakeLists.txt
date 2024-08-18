cmake_minimum_required(VERSION 3.14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(
	EKF_standalone
	VERSION 1.0
	LANGUAGES CXX
)

## COMPILE OPTIONS ##
add_compile_options(
	-std=gnu++20
)

# Eigen needs to be installed on the system
find_package(Eigen3 REQUIRED)
link_directories(${Eigen_INCLUDE_DIRS})

# Include directory that builds EKF library
add_subdirectory(src)

add_executable(runEKF
	apps/runEKF.cpp
)

target_link_libraries(runEKF
	ekf
	${Eigen_LIBRARIES}
)

