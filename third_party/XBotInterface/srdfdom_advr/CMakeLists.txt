cmake_minimum_required(VERSION 3.0)
project(srdfdom_advr)

find_package(Boost REQUIRED)
#find_package(rosconsole_bridge REQUIRED)
find_package(urdf REQUIRED)
find_package(cmake_modules)
#find_package(TinyXML REQUIRED)

add_library(${PROJECT_NAME} STATIC src/model.cpp)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    #${TinyXML_LIBRARIES}
    #${rosconsole_brigde_LIBRARIES}
    ${urdf_LIBRARIES})
target_include_directories(${PROJECT_NAME} 
                           PUBLIC 
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           )
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

#install(DIRECTORY include/${PROJECT_NAME}
#    DESTINATION include)

library_install(${PROJECT_NAME} 0 0 1)

