cmake_minimum_required(VERSION 3.10)

# Project Name
project(PointProcessing)

# Set C++ standard to C++17
# set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release for optimization
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

# Specify the macOS SDK path
set(CMAKE_OSX_SYSROOT /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk)

set(CMAKE_INSTALL_RPATH "/Applications/MATLAB_R2023b.app/extern/bin/maci64")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,/Applications/MATLAB_R2023b.app/extern/bin/maci64")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Specify the target architecture (x86_64)
set(CMAKE_OSX_ARCHITECTURES x86_64)

file(COPY /Applications/MATLAB_R2023b.app/extern/bin/maci64/libMatlabEngine.dylib
     DESTINATION ${CMAKE_BINARY_DIR})

# Hint CMake to find CGAL
set(CGAL_DIR "/usr/local/Cellar/cgal/5.6/lib/cmake/CGAL")

# Find CGAL
find_package(CGAL REQUIRED)
# include_directories("/usr/local/Cellar/cgal/5.6/include")

# Find Boost
find_package(Boost 1.83.0 REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
# include_directories("/usr/local/Cellar/boost/1.83.0/include")

# Hint CMake to find Boost
set(BOOST_ROOT "/usr/local/Cellar/boost/1.83.0")

# Find Matlab
find_package(Matlab REQUIRED)
include_directories(${Matlab_INCLUDE_DIRS})

# Add MATLAB's include and lib paths
include_directories(/Applications/MATLAB_R2023b.app/extern/include)
link_directories(/Applications/MATLAB_R2023b.app/bin/maci64)

matlab_add_mex(
    NAME PointProcessing
    SRC /Users/ciaratorguson/point_processing/test_includes.cpp
    # SRC /Users/ciaratorguson/point_processing/main.cpp
    OUTPUT_NAME PointProcessing
    LINK_TO ${Matlab_LIBRARIES}
)

# Set properties for the MEX target
set_target_properties(PointProcessing PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/mex"
)

# Link Boost libraries
# target_link_libraries(PointProcessing CGAL Boost)
# target_link_libraries(PointProcessing ${Boost_LIBRARIES})
target_include_directories(PointProcessing PRIVATE ${Matlab_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

message(STATUS "CMake Binary Directory: ${CMAKE_BINARY_DIR}")
message(STATUS "MEX Output Name: ${PointProcessing}")
message(STATUS "MEX Output Directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "MATLAB Found: ${Matlab_FOUND}")
message(STATUS "MATLAB Include Dirs: ${Matlab_INCLUDE_DIRS}")
message(STATUS "MATLAB Libraries: ${Matlab_LIBRARIES}")
message(STATUS "MATLAB MEX Library: ${Matlab_MEX_LIBRARY}")
message(STATUS "MATLAB Engine Library: ${Matlab_ENGINE_LIBRARY}")


