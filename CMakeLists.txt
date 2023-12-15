cmake_minimum_required(VERSION 3.10)

# Project Name
project(PointProcessing)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build type to Release for optimization
set(CMAKE_BUILD_TYPE Release)

# Specify the macOS SDK path
set(CMAKE_OSX_SYSROOT /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk)

# Find CGAL
find_package(CGAL REQUIRED)

# Find Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Add main executable
# add_executable(main main.cpp)
add_executable(test_includes test_includes.cpp)

# If you are using CGAL, find_package(CGAL) might be necessary
# find_package(CGAL REQUIRED)
