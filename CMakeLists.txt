# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(opencv_example_project)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package(Threads)
SET(CMAKE_BUILD_TYPE "Debug") 

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# message("The vtk DIR:"${VTK_INCLUDE_DIRS})
# Declare the executable target built from your sources
# add_executable(opencv_example quaternion.cpp)
add_executable(opencv_example test.cpp)

# Link your application with OpenCV libraries
target_link_libraries(opencv_example ${OpenCV_LIBS})

target_link_libraries(opencv_example ${GTEST_LIBRARIES}) # Replace ${PROJECT_NAME} with your target name

target_link_libraries(opencv_example ${CMAKE_THREAD_LIBS_INIT})
