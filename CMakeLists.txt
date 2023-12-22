project(lifelong)

cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  # SET(CMAKE_BUILD_TYPE "DEBUG")
  set(CMAKE_BUILD_TYPE "RELEASE")
  # SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO") SET(CMAKE_BUILD_TYPE "MINSIZEREL")
endif()

include_directories("inc")
# file(GLOB SOURCES "src/*.cpp")
set(RHCR_SRCS
    src/BasicGraph.cpp
    src/BasicSystem.cpp
    src/BeeGraph.cpp
    src/BeeSystem.cpp
    src/common.cpp src/ECBS.cpp
    src/ECBSNode.cpp
    src/ID.cpp
    src/KivaGraph.cpp
    src/KivaSystem.cpp
    src/LRAStar.cpp
    src/MAPFSolver.cpp
    src/OnlineGraph.cpp
    src/OnlineSystem.cpp
    src/PathTable.cpp
    src/PBS.cpp
    src/PBSNode.cpp
    src/PriorityGraph.cpp
    src/ReservationTable.cpp
    src/SingleAgentSolver.cpp
    src/SIPP.cpp
    src/SortingGraph.cpp
    src/SortingSystem.cpp
    src/States.cpp
    src/StateTimeAStar.cpp
    src/WHCAStar.cpp)

set(DRIVER_SRCS src/driver.cpp)
set(SIPP_DEMO_DRIVER_SRCS src/sipp_demo.cpp)

# Find Boost
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(lifelong ${RHCR_SRCS} ${DRIVER_SRCS})
target_link_libraries(lifelong ${Boost_LIBRARIES})

add_executable(sipp_demo ${RHCR_SRCS} ${SIPP_DEMO_DRIVER_SRCS})
target_link_libraries(sipp_demo ${Boost_LIBRARIES})
target_compile_options(sipp_demo PRIVATE -O2 -g -fsanitize=address -fsanitize=undefined)
target_link_options(sipp_demo PRIVATE -fsanitize=address -fsanitize=undefined)
