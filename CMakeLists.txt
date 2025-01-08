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

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(wolf_msgs REQUIRED)
find_package(wolf_controller_utils REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(
    include
    /opt/ros/$ENV{ROS_DISTRO}/include/kdl_parser/  # FIX FOR ROS HUMBLE
    /opt/ros/$ENV{ROS_DISTRO}/include/realtime_tools/ # FIX FOR ROS HUMBLE
    /opt/ros/$ENV{ROS_DISTRO}/include/interactive_markers/ # FIX FOR ROS HUMBLE
    third_party/MatLogger2/include
    third_party/XBotInterface/include
    third_party/XBotInterface/srdfdom_advr/include
    third_party/ModelInterfaceRBDL/include
    third_party/OpenSoT/include
    ${EIGEN3_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${wolf_msgs_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
)

# ADVR Packages
# Include ADVR packages as subdirectories
add_subdirectory(third_party/MatLogger2)
add_subdirectory(third_party/XBotInterface)
add_subdirectory(third_party/ModelInterfaceRBDL)
add_subdirectory(third_party/OpenSoT)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if(${catkin_FOUND})
    set(TASK_WRAPPERS_FOLDER "task_ros_wrappers")
    catkin_package(
      INCLUDE_DIRS
        include
        third_party/XBotInterface/include
        third_party/ModelInterfaceRBDL/include
        third_party/OpenSoT/include
      LIBRARIES
        quadruped_robot
        cartesian_trajectory
        id_problem
        task_wrappers
        OpenSoT
        OpenSotBackEndQPOases
        OpenSotBackEndeiQuadProg
        XBotInterface
        XBotCoreModel
        ModelInterfaceRBDL
        matlogger2
      CATKIN_DEPENDS
        wolf_msgs
        wolf_controller_utils
        interactive_markers
        realtime_tools
    )
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(TASK_WRAPPERS_FOLDER "task_ros2_wrappers")
    ament_export_include_directories(include)
    ament_export_dependencies(wolf_msgs wolf_controller_utils interactive_markers realtime_tools)
    ament_export_libraries(
        quadruped_robot
        cartesian_trajectory
        id_problem
        task_wrappers
        OpenSoT
        OpenSotBackEndQPOases
        OpenSotBackEndeiQuadProg
        XBotInterface
        XBotCoreModel
        ModelInterfaceRBDL
        matlogger2)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

add_library(quadruped_robot SHARED
    src/quadruped_robot.cpp)
target_link_libraries(quadruped_robot
    ${wolf_controller_utils_LIBRARIES}
    OpenSoT
    XBotInterface
    XBotCoreModel
    ModelInterfaceRBDL)

add_library(cartesian_trajectory SHARED
    src/cartesian_trajectory.cpp)
target_link_libraries(cartesian_trajectory
    OpenSoT)

add_library(task_wrappers SHARED
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
    ${tf2_ros_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    ${realtime_tools_LIBRARIES}
    ${wolf_controller_utils_LIBRARIES}
    ${wolf_msgs_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    OpenSoT
    XBotInterface
    XBotCoreModel
    ModelInterfaceRBDL)

add_library(id_problem SHARED src/id_problem.cpp)
target_link_libraries(id_problem
    quadruped_robot 
    task_wrappers 
    OpenSoT
    XBotInterface
    XBotCoreModel
    ModelInterfaceRBDL)

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

#install(
#  DIRECTORY third_party/XBotInterface/include/
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
#)
#install(
#  DIRECTORY third_party/ModelInterfaceRBDL/include/
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
#)
#install(
#  DIRECTORY third_party/OpenSoT/include/
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
#)

install(
  TARGETS
    quadruped_robot
    cartesian_trajectory
    id_problem
    task_wrappers
    OpenSoT
    OpenSotBackEndQPOases
    OpenSotBackEndeiQuadProg
    XBotInterface
    XBotCoreModel
    ModelInterfaceRBDL
    matlogger2
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
