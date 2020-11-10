cmake_minimum_required(VERSION 3.5)
project(SIMBICON)

#================flags settings==============
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(STATUS "Build type: debug")
    add_definitions("-O0 -g -Wall -fPIC -ggdb ")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Build type: release")
    add_definitions("-O3 -g -fPIC")
else()
    message(STATUS "Build type: release(default)")
    add_definitions("-O3 -fPIC")
endif()

#================include path===============
include_directories("./include")
include_directories("./")
include_directories("./ode-0.9/include")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#==================dependency================
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    include_directories(${OpenGL_INCLUDE_DIRS})
    link_directories(${OpenGL_LIBRARY_DIRS})
    add_definitions(${OpenGL_DEFINITIONS})
endif(OPENGL_FOUND)

#================subdirectory==========
add_subdirectory(gsl)
add_subdirectory(Utils)
add_subdirectory(MathLib) 
add_subdirectory(GLUtils) 
add_subdirectory(ode-0.9)
add_subdirectory(Physics)
add_subdirectory(Core)
add_subdirectory(AppGUI) 
