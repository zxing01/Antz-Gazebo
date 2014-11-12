cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

cmake_policy(SET CMP0042 NEW)

#set(GCC_COMPILE_FLAGS "-Wno-c++11-extensions")
#set(GCC_LINK_FLAGS)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

find_package(Protobuf REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(antz SHARED Antz.cc AntzPlugin.cc AntzState.cc Walker.cc Beacon.cc Explorer.cc Avoider.cc)
target_link_libraries(antz ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})
add_library(factory SHARED Antz.cc FactoryPlugin.cc)
target_link_libraries(factory ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})
add_library(colorful SHARED Antz.cc ColorfulPlugin.cc)
target_link_libraries(colorful ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})