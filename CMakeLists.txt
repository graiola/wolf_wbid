cmake_minimum_required(VERSION 3.5)
project(wolf_wbid)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(ModelInterfaceRBDL REQUIRED)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if(${catkin_FOUND})
    set(TASK_WRAPPERS_FOLDER "task_ros_wrappers")
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES quadruped_robot cartesian_trajectory id_problem task_wrappers
      CATKIN_DEPENDS wolf_msgs wolf_controller_utils interactive_markers
    )
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(TASK_WRAPPERS_FOLDER "task_ros2_wrappers")
    ament_export_include_directories(include)
    ament_export_libraries(quadruped_robot cartesian_trajectory id_problem task_wrappers)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

find_package(tf2 REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(wolf_msgs REQUIRED)
find_package(wolf_controller_utils REQUIRED)
find_package(interactive_markers REQUIRED)

include_directories(
    include
    ${tf2_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${wolf_msgs_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS}
)

add_library(quadruped_robot
    src/quadruped_robot.cpp)
target_link_libraries(quadruped_robot
    ${wolf_controller_utils_LIBRARIES}
    ${OpenSoT_LIBRARIES}
    ${XBotInterface_LIBRARIES}
    ${ModelInterfaceRBDL_LIBRARIES})

add_library(cartesian_trajectory
    src/cartesian_trajectory.cpp)
target_link_libraries(cartesian_trajectory
    ${OpenSoT_LIBRARIES})

add_library(task_wrappers
    src/task_interface.cpp
    src/${TASK_WRAPPERS_FOLDER}/handler.cpp
    src/${TASK_WRAPPERS_FOLDER}/cartesian.cpp
    src/${TASK_WRAPPERS_FOLDER}/com.cpp
    src/${TASK_WRAPPERS_FOLDER}/momentum.cpp
    src/${TASK_WRAPPERS_FOLDER}/postural.cpp
    src/${TASK_WRAPPERS_FOLDER}/wrench.cpp
    )
target_link_libraries(task_wrappers 
    cartesian_trajectory 
    quadruped_robot
    ${tf2_LIBRARIES}
    ${realtime_tools_conversions_LIBRARIES}
    ${wolf_controller_utils_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${OpenSoT_LIBRARIES}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS})

add_library(id_problem src/id_problem.cpp)
target_link_libraries(id_problem 
    quadruped_robot 
    task_wrappers 
    ${OpenSoT_LIBRARIES}
    ${XBotInterface_LIBRARIES} 
    ${ModelInterfaceRBDL_LIBRARIES})

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(
  TARGETS quadruped_robot cartesian_trajectory id_problem task_wrappers
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
