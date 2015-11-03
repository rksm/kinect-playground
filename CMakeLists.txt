cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project (kinect-test)

set(CMAKE_BUILD_TYPE Debug)
find_program(CCACHE_FOUND ccache) 
if(CCACHE_FOUND) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif(CCACHE_FOUND) 

# set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --save-temps -fverbose-asm -Os -Wall -Wno-strict-aliasing -Wcast-align -Wno-cast-align -fdata-sections -ffunction-sections -fno-align-loops -fno-move-loop-invariants -fno-peel-loops -fno-prefetch-loop-arrays -fno-rerun-cse-after-loop -fno-reschedule-modulo-scheduled-loops -fno-tree-loop-im -fno-tree-loop-ivcanon -fno-tree-loop-linear -fno-tree-loop-optimize -fno-tree-vect-loop-version -fno-unroll-all-loops -fno-unroll-loops -fno-loop-optimize -fshort-enums")

# Profiling
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# libs

find_package(PCL 1.7 REQUIRED geometry io visualization)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(freenect2 REQUIRED)
find_package (OpenCV REQUIRED)
#find_package (Jsoncpp REQUIRED)

# link_directories(/usr/local/lib) 
# include_directories(${freenect2_INCLUDE_DIR})
# 

include_directories(${src})

set(kinect-test_SOURCES
  src/pcl.cpp
  src/opencv.cpp
  src/freenect.cpp
  src/main.cpp)

add_executable(kinect-test ${kinect-test_SOURCES})

target_link_libraries(kinect-test ${PCL_LIBRARIES})
target_link_libraries (kinect-test ${freenect2_LIBRARIES})
target_link_libraries (kinect-test ${OpenCV_LIBS})
#target_link_libraries (kinect-test ${Jsoncpp_LIBRARY})

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# compiler features / version
set(needed_features cxx_strong_enums cxx_constexpr cxx_auto_type)
target_compile_features(kinect-test PRIVATE ${needed_features})
