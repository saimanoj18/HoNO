cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(new_keypoint_detector)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#find_package(OpenMP REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")



# Adding PCL library
#set(PCL_DIR "/home/sai/workspace/pcl-pcl-1.7.2/build/")
find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (dataset_evaluation_HoNO dataset_evaluation_HoNO.cpp)
target_link_libraries (dataset_evaluation_HoNO ${PCL_LIBRARIES} )


add_executable (HoNO HoNO.cpp)
target_link_libraries (HoNO ${PCL_LIBRARIES} )





















