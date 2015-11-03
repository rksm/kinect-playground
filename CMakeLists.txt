cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project (kinect-test)

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# libs

set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.7 REQUIRED geometry io visualization)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(freenect2 REQUIRED)
find_package (OpenCV REQUIRED)
#find_package (Jsoncpp REQUIRED)

link_directories(/usr/local/lib) 
# include_directories(${freenect2_INCLUDE_DIR})

set(kinect-test_SOURCES src/main.cpp)

add_executable(kinect-test ${kinect-test_SOURCES})

target_link_libraries(kinect-test ${PCL_LIBRARIES})
target_link_libraries (kinect-test ${freenect2_LIBRARIES})
target_link_libraries (kinect-test ${OpenCV_LIBS})
#target_link_libraries (kinect-test ${Jsoncpp_LIBRARY})

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# compiler features / version
set(needed_features cxx_strong_enums cxx_constexpr cxx_auto_type)
target_compile_features(kinect-test PRIVATE ${needed_features})