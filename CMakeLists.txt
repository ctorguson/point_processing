cmake_minimum_required(VERSION 3.10) 

project(PointProcessing)

# Set the compiler
set(CMAKE_OSX_SYSROOT "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk")

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

# Set build type to Release for performance optimization
set(CMAKE_BUILD_TYPE Release)

# Set C++ standard to C++
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Specify the target architectur# Target x86_64
set(CMAKE_OSX_ARCHITECTURES x86_64) 

# Path to the vcpkg toolchain file
set(CMAKE_TOOLCHAIN_FILE "/Users/ciaratorguson/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

# Find necessary packages using vcpkg
unset(CGAL_ROOT CACHE)
find_package(CGAL 5.6 CONFIG REQUIRED) 

# Include directories for CGAL 
include_directories(/usr/local/Cellar/cgal/5.6/include)
include_directories(${CGAL_INCLUDE_DIRS}) 
include_directories(/usr/local/lib/cmake/CGAL/include)
include_directories("./buildtrees/cgal/src/v5.6-e124d30c96.clean/Kernel_23/include") 
include_directories(/Users/ciaratorguson/point_processing)

# Link directories for MATLAB
include_directories(/Users/ciaratorguson/Desktop/matlab/extern/include)
link_directories(/Users/ciaratorguson/Desktop/matlab/bin/maci64)
include_directories(/Applications/MATLAB_R2023b.app/extern/include)

# Specify the target and source files for the executable
add_executable(PointProcessing /Users/ciaratorguson/point_processing/main.cpp)
add_executable(main main.cpp)

# Linking necessary libraries
target_link_libraries(PointProcessing PRIVATE CGAL::CGAL ${MATLAB_LIBRARIES})
target_link_libraries(main ${CGAL_LIBRARIES})

# Set the output name to have the appropriate MEX extension for MATLAB
set_target_properties(PointProcessing PROPERTIES PREFIX "" SUFFIX ".mexmaci64")