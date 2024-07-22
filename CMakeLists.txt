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

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if(${catkin_FOUND})
    find_package(catkin REQUIRED COMPONENTS tf realtime_tools rt_logger wolf_msgs wolf_controller_utils)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES quadruped_robot cartesian_trajectory id_problem task_wrappers
      CATKIN_DEPENDS wolf_msgs wolf_controller_utils
    )
    add_definitions(-DROS)
elseif(${ament_cmake_FOUND})
    ament_export_include_directories(include)
    ament_export_libraries(quadruped_robot cartesian_trajectory id_problem task_wrappers)
    ament_package()
    add_definitions(-DROS2)
endif()

# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(ModelInterfaceRBDL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS}
)

add_library(quadruped_robot
    src/quadruped_robot.cpp)
target_link_libraries(quadruped_robot ${OpenSoT_LIBRARIES} ${XBotInterface_INCLUDE_DIRS} ${ModelInterfaceRBDL_INCLUDE_DIRS})

add_library(cartesian_trajectory
    src/cartesian_trajectory.cpp)
target_link_libraries(cartesian_trajectory ${OpenSoT_LIBRARIES})

add_library(task_wrappers
    src/task_interface.cpp
    src/task_ros_wrappers/handler.cpp
    src/task_ros_wrappers/cartesian.cpp
    src/task_ros_wrappers/com.cpp
    src/task_ros_wrappers/momentum.cpp
    src/task_ros_wrappers/postural.cpp
    src/task_ros_wrappers/wrench.cpp)
target_link_libraries(task_wrappers cartesian_trajectory ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES})

add_library(id_problem
    src/id_problem.cpp)
target_link_libraries(id_problem quadruped_robot task_wrappers ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES}
    ${XBotInterface_LIBRARIES} ${ModelInterfaceRBDL_LIBRARIES})

install(TARGETS quadruped_robot cartesian_trajectory task_wrappers id_problem
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
