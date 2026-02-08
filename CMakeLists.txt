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
    external/rbdl/include
    external/qpOASES/include
    ${EIGEN3_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${wolf_msgs_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
)

link_directories(/opt/ros/$ENV{ROS_DISTRO}/lib/) # FIX FOR ROS HUMBLE

# ADVR Packages
# (third_party removed)

# Solvers
add_subdirectory(external/rbdl)
add_subdirectory(external/eiQuadProg)
add_subdirectory(external/qpOASES)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if(${catkin_FOUND})
    set(TASK_WRAPPERS_FOLDER "ros")
    catkin_package(
      INCLUDE_DIRS
        include
        external/rbdl/include
        external/eiQuadProg/include
        external/qpOASES/include
      LIBRARIES
        quadruped_robot
        cartesian_trajectory
        id_problem
        task_wrappers
        rbdl
        rbdl_urdfreader
        eiQuadProg
        qpOASES
      CATKIN_DEPENDS
        wolf_msgs
        wolf_controller_utils
        interactive_markers
        realtime_tools
        kdl_parser
    )
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(TASK_WRAPPERS_FOLDER "ros2")
    ament_export_include_directories(
        include
        external/rbdl/include
        external/eiQuadProg/include
        external/qpOASES/include)
    ament_export_dependencies(wolf_msgs wolf_controller_utils interactive_markers realtime_tools kdl_parser)
    ament_export_libraries(
        quadruped_robot
        cartesian_trajectory
        id_problem
        task_wrappers
        rbdl
        rbdl_urdfreader
        eiQuadProg
        qpOASES)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

add_library(quadruped_robot SHARED
    src/core/quadruped_robot_rbdl.cpp)
target_link_libraries(quadruped_robot
    rbdl
    rbdl_urdfreader
    ${wolf_controller_utils_LIBRARIES})

add_library(cartesian_trajectory SHARED
    src/core/cartesian_trajectory.cpp)
target_link_libraries(cartesian_trajectory)

add_library(task_wrappers SHARED
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
    interactive_markers # FIX FOR ROS HUMBLE
    realtime_tools # FIX FOR ROS HUMBLE
    )

add_library(id_problem SHARED 
    src/core/id_problem.cpp
    src/core/task_interface.cpp
    src/wbid/id_variables.cpp
    src/wbid/qp/eiquadprog_solver.cpp
    src/wbid/qp/qpoases_solver.cpp
    src/wbid/qp/qp_solver_factory.cpp
    src/wbid/constraints/friction_cone_constraint.cpp
    src/wbid/constraints/friction_cones_constraint.cpp
    src/wbid/constraints/contact_force_bounds_constraint.cpp
    src/wbid/constraints/torque_limits_constraint.cpp
    src/wbid/constraints/dynamics_equality_constraint.cpp
    src/wbid/constraints/base_accel_z_constraint.cpp
    src/wbid/tasks/task_base.cpp
    src/wbid/tasks/angular_momentum_task.cpp
    src/wbid/tasks/cartesian_task.cpp
    src/wbid/tasks/com_task.cpp
    src/wbid/tasks/postural_task.cpp
    src/wbid/tasks/wrench_task.cpp
)
target_link_libraries(id_problem
    quadruped_robot 
    task_wrappers 
    eiQuadProg
    qpOASES)

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(TARGETS
        rbdl
        rbdl_urdfreader
        quadruped_robot
        cartesian_trajectory
        id_problem
        task_wrappers
        eiQuadProg
        qpOASES
        DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib
)
