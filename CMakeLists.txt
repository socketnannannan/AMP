cmake_minimum_required(VERSION 3.5)
project(map_io)

add_definitions(-std=c++14)
add_compile_options(-std=c++14)
# 07. add version.h file
set(PROJECT_VERSION "v10.2.3")
set(PROJECT_VERSION_MAJOR 10)
set(PROJECT_VERSION_MINOR 2)
set(PROJECT_VERSION_PATCH 3)
set(BUILD_TIME "20240115")
set(GIT_HASH 0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE "Debug")

# set(CMAKE_BUILD_TYPE Release)
configure_file(version.h.in version.h)

configure_file (
        "${PROJECT_SOURCE_DIR}/version.h.in"
        "${PROJECT_SOURCE_DIR}/include/version.h"
)

option(OFFLINE_DEBUG "Build offline_debug on PC" ON)
message("OFFLINE_DEBUG is ${OFFLINE_DEBUG}")

option(BUILD_IN_TDA4 "Build the hpa_slam on tda4." OFF)
message("BUILD_IN_TDA4 is ${BUILD_IN_TDA4}")

option(WITH_ROS "Build the hpa_slam on PC WITH_ROS." OFF)
message("WITH ROS is ${WITH_ROS}")

option(BUILD_STATIC_LIB "on == Build .a lib; off == build .so lib" OFF)

if(BUILD_STATIC_LIB)
    message("building libmap_io.a ...")
else()
    message("building libmap_io.so ... ")
endif()


if (BUILD_IN_TDA4) # tda4运行
#     set(SDK_PATH "../..")
    set(USE_DESAY "Build the project using DESAY Interface" True)
    add_definitions("-DUSE_DESAY")

    find_package(Eigen3 3.3 REQUIRED
            PATHS "${SDK_PATH}/share/cmake"
            NO_DEFAULT_PATH)

    message("LIBRARY_OUTPUT_PATH: ${PROJECT_SOURCE_DIR}/lib/tda4")
    set(LIB_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/lib/tda4)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})

    link_directories(${PROJECT_SOURCE_DIR}/src/path_process/lib_qnx/lib)

    include_directories(
            ${EIGEN3_INCLUDE_DIR}
            include
            ${SDK_PATH}/include
            src/path_process
            src/path_process/lib_qnx/include
    )

    if(BUILD_STATIC_LIB)
        add_library(map_io_static STATIC
                    src/map_io.cpp
                    src/path_process/path_process.cc
        )
        target_link_libraries(map_io_static PUBLIC libosqp.a)

        set_target_properties(map_io_static PROPERTIES OUTPUT_NAME "map_io")

        add_executable(read example/read.cpp src/map_io.cpp)
        add_executable(write example/write.cpp src/map_io.cpp)

        # set(CMAKE_INSTALL_LIBDIR ${PROJECT_SOURCE_DIR}/lib/tda4)
        # set(CMAKE_INSTALL_BINDIR ${PROJECT_SOURCE_DIR}/bin)

        #     install(TARGETS ${PROJECT_NAME}
        #             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        #             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        #             ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

        install(TARGETS read
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        install(TARGETS write
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        install(TARGETS map_io_static LIBRARY DESTINATION lib)
    else()
        add_library(map_io SHARED
                    src/map_io.cpp
                    src/path_process/path_process.cc
                )
        target_link_libraries(map_io PUBLIC libosqp.a)

        add_executable(read example/read.cpp src/map_io.cpp)
        add_executable(write example/write.cpp src/map_io.cpp)

        set(CMAKE_INSTALL_LIBDIR ${PROJECT_SOURCE_DIR}/lib/tda4)
        set(CMAKE_INSTALL_BINDIR ${PROJECT_SOURCE_DIR}/bin)

        #     install(TARGETS ${PROJECT_NAME}
        #             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        #             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        #             ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

        install(TARGETS read
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        install(TARGETS write
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
        install(TARGETS map_io LIBRARY DESTINATION lib)
    endif()

elseif (WITH_ROS) # pc版或tda4版ros
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            roslib
            common_msgs
            )
    include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/map_io)

    include_directories(${catkin_INCLUDE_DIRS}
    include
    src/path_process
    src/path_process/lib_qnx/include
    )

    link_directories(${catkin_LIBRARY_DIRS}
    ${PROJECT_SOURCE_DIR}/src/path_process/lib_lnx/osqp/lib)

    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



    catkin_package()

    message("LIBRARY_OUTPUT_PATH: ${PROJECT_SOURCE_DIR}/lib/ros")
    set(LIB_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/lib/ros)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
    add_library(map_io SHARED
                src/map_io.cpp
                src/path_process/path_process.cc
            )

    add_executable(read example/read.cpp src/map_io.cpp)
    add_executable(write example/write.cpp src/map_io.cpp)  

    target_link_libraries(map_io ${catkin_LIBRARIES})
    target_link_libraries(read ${catkin_LIBRARIES})
    target_link_libraries(write ${catkin_LIBRARIES})

else()
    set(LIB_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/lib/pc_debug)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})
    message("LIBRARY_OUTPUT_PATH: " ${LIB_OUTPUT_DIR})

    include_directories(${catkin_INCLUDE_DIRS}
        include
        src/path_process
        src/path_process/lib_qnx/include
        /opt/ros/noetic/include
    )

    link_directories(
        ${PROJECT_SOURCE_DIR}/src/path_process/lib_lnx/osqp/lib
    )

    add_library(map_io SHARED
        src/map_io.cpp
        src/path_process/path_process.cc
    )

    target_link_libraries(map_io 
        libosqp.a
    )

    if (OFFLINE_DEBUG) 
        find_package(Eigen3 REQUIRED)
        find_package(Boost COMPONENTS filesystem)

        include_directories(
                offline_debug  #add debug
                /usr/include/python3.8
        )
        add_executable(offline_debug 
            offline_debug/offline_debug.cpp
        )
        target_link_libraries(offline_debug 
            map_io
            ${Boost_LIBRARIES}
            python3.8
            /opt/ros/noetic/lib/libroslib.so
        )

        # install(TARGETS offline_debug RUNTIME DESTINATION bin)
    endif ()
endif ()
