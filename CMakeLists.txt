cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()

project(CreateNormals)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )
 
include_directories(C:/opencv/build)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CreateNormals_SOURCE_DIR})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE CreateNormals_HEADERS *.h)
file(GLOB_RECURSE CreateNormals_SOURCES *.cpp)
set (CreateNormals_INCLUDE_DIRS "")
foreach (_headerFile ${CreateNormals_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND CreateNormals_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES CreateNormals_INCLUDE_DIRS)

include_directories(${CreateNormals_INCLUDE_DIRS})


add_Library (CreateNormals CreateNormals.cpp ConnectedComponents.cpp ${CreateNormals_HEADERS})
target_link_libraries (CreateNormals ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (CreateNormalsMain CreateNormalsMain.cpp ${CreateNormals_HEADERS})
target_link_libraries (CreateNormalsMain CreateNormals)

