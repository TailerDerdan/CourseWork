cmake_minimum_required(VERSION 3.14)
project(MultiRobotMotionPlanning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 3.0 COMPONENTS Graphics Window System REQUIRED)

add_library(MultiRobotMotionPlanningLib
        src/graph/Graph.cpp
        src/graph/Graph.h
        src/robot/Robot.cpp
        src/robot/Robot.h
        src/dynamicEnvironments/IDynamicEnvironments.h
        src/robot/IRobot.h
        src/utils/Types.h
        src/utils/Constants.h
        src/staticEnvironments/StaticEnvironments.cpp
        src/staticEnvironments/StaticEnvironments.h
        src/utils/utilFunc.h
)

target_link_libraries(MultiRobotMotionPlanningLib PUBLIC
        SFML::Graphics
        SFML::Window
        SFML::System
)

target_include_directories(MultiRobotMotionPlanningLib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(multiRobotMotionPlanning src/main.cpp)
target_link_libraries(multiRobotMotionPlanning PRIVATE MultiRobotMotionPlanningLib)

#add_subdirectory(CShapesTests)

#target_compile_definitions(CShapesTests PRIVATE
#        TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

