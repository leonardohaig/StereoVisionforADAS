cmake_minimum_required(VERSION 3.1)
project( StereoVisionforADAS )

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(./ SRC_DIR)

add_executable( StereoVisionforADAS ${SRC_DIR})

target_link_libraries( StereoVisionforADAS ${OpenCV_LIBS} )