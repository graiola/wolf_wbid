cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

# compite GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# gTests TODO add libs
SET(TestLibs XBotInterface ${GTEST_BOTH_LIBRARIES})

add_definitions(-DXBOT_TESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
add_definitions(-DXBOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/robots/")

# Unit tests related to Joint class
add_executable(testJoint     testJoint.cpp)
target_link_libraries(testJoint ${TestLibs})
add_dependencies(testJoint GTest-ext XBotInterface)
add_test(NAME XBot_testJoint COMMAND testJoint)

# Unit tests related to KinematicChain class
add_executable(testKinematicChain     testKinematicChain.cpp)
target_link_libraries(testKinematicChain ${TestLibs})
add_dependencies(testKinematicChain GTest-ext XBotInterface)
add_test(NAME XBot_testKinematicChain COMMAND testKinematicChain)

# Unit tests related to XbotInterface class
add_executable(testXBotInterface     testXBotInterface.cpp)
target_link_libraries(testXBotInterface ${TestLibs})
add_dependencies(testXBotInterface GTest-ext XBotInterface)
add_test(NAME XBot_testXBotInterface COMMAND testXBotInterface)

# Unit tests related to ModelInterface class
add_executable(testModelInterface     testModelInterface.cpp)
target_link_libraries(testModelInterface ${TestLibs})
add_dependencies(testModelInterface GTest-ext XBotInterface)
add_test(NAME XBot_testModelInterface COMMAND testModelInterface)

# Unit tests related to Utils
add_executable(testUtils    testUtils.cpp)
target_link_libraries(testUtils ${TestLibs})
add_dependencies(testUtils GTest-ext)
add_test(NAME XBot_testUtils COMMAND testUtils)
