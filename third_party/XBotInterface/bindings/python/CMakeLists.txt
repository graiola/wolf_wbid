cmake_minimum_required(VERSION 3.5)

find_package(pybind11 QUIET)

add_compile_options(-std=c++14)

option(COMPILE_PYTHON_BINDINGS "Compile python bindings (requires pybind11)" TRUE)

if(${pybind11_FOUND} AND ${COMPILE_PYTHON_BINDINGS})
    
    message(STATUS "Will compile python bindings")
    
    # get python install path 
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "if True:
            from distutils import sysconfig as sc
            print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Python install dir: ${PYTHON_SITE}")
    
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    
    pybind11_add_module(config_options pyConfigOptions.cpp)
    target_link_libraries(config_options PUBLIC XBotInterface)
    install(TARGETS config_options
            DESTINATION ${PYTHON_SITE}/xbot_interface)
    
    pybind11_add_module(xbot_interface pyXBotInterface.cpp)
    target_link_libraries(xbot_interface PUBLIC XBotInterface)
    install(TARGETS xbot_interface
            DESTINATION ${PYTHON_SITE}/xbot_interface)
            
    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES  ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_SITE}/xbot_interface)

    add_custom_target(pybind_build_2_3
        COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/compile_py_2_3.py ${CMAKE_BINARY_DIR})

    add_custom_target(pybind_build_2_3_install
        COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/compile_py_2_3.py ${CMAKE_BINARY_DIR} install)

            
else()
    message(STATUS "Pybind11 bindings won't be available")
endif()
