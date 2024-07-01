cmake_minimum_required(VERSION 3.1.15)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# opencv required
if(${OpenCV_FOUND})
    add_subdirectory(Sample-ColorViewer)
    add_subdirectory(Sample-DepthViewer)
    add_subdirectory(Sample-PointCloud)
endif(${OpenCV_FOUND})

