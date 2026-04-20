CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
	LINKER_LANGUAGE CXX
)

INCLUDE_DIRECTORIES (
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
	${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/
)

SET ( URDFREADER_SOURCES
	urdfreader.cc
	)

SET (RBDL_USE_ROS_URDF_LIBRARY FALSE)
SET (URDFREADER_COMPILE_DEFINITIONS "")

IF (DEFINED ENV{ROS_ROOT})
	MESSAGE (STATUS "ROS found: $ENV{ROS_ROOT}")
	find_package(catkin REQUIRED COMPONENTS urdf)
	SET (RBDL_USE_ROS_URDF_LIBRARY ON)
	include_directories(${catkin_INCLUDE_DIRS})
ELSE ()
	find_package(urdf QUIET)
	if (urdf_FOUND)
		MESSAGE (STATUS "ROS 2 urdf package found, using system URDF parser")
		SET (RBDL_USE_ROS_URDF_LIBRARY ON)
	else ()
		find_package(urdfdom_headers QUIET)
		find_package(urdfdom QUIET)
		if (urdfdom_headers_FOUND AND urdfdom_FOUND)
			MESSAGE (STATUS "urdfdom found, using system URDF parser")
			SET (RBDL_USE_ROS_URDF_LIBRARY ON)
		endif ()
	endif ()
ENDIF ()

IF (RBDL_USE_ROS_URDF_LIBRARY)
	LIST(APPEND URDFREADER_COMPILE_DEFINITIONS RBDL_USE_ROS_URDF_LIBRARY)
	SET (URDFREADER_DEPENDENCIES
		rbdl
		)
	if (DEFINED ENV{ROS_ROOT})
		LIST (APPEND URDFREADER_DEPENDENCIES ${urdf_LIBRARIES})
	elseif (TARGET urdf::urdf)
		LIST (APPEND URDFREADER_DEPENDENCIES urdf::urdf)
	elseif (TARGET urdfdom::urdf_parser)
		LIST (APPEND URDFREADER_DEPENDENCIES
			urdfdom::urdf_parser
			urdfdom::urdfdom_model
			urdfdom::urdfdom_world
			urdfdom_headers::urdfdom_headers)
	endif ()
ELSE()
	SET (URDFREADER_SOURCES
		${URDFREADER_SOURCES}
		thirdparty/urdf/urdfdom/urdf_parser/src/check_urdf.cpp	
		thirdparty/urdf/urdfdom/urdf_parser/src/pose.cpp
		thirdparty/urdf/urdfdom/urdf_parser/src/model.cpp
		thirdparty/urdf/urdfdom/urdf_parser/src/link.cpp
		thirdparty/urdf/urdfdom/urdf_parser/src/joint.cpp
		thirdparty/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
		thirdparty/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h	
		thirdparty/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
		thirdparty/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
		thirdparty/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
		thirdparty/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
		thirdparty/tinyxml/tinystr.cpp
		thirdparty/tinyxml/tinyxml.cpp
                thirdparty/tinyxml/tinyxmlerror.cpp
		thirdparty/tinyxml/tinyxmlparser.cpp
		thirdparty/urdf/boost_replacement/lexical_cast.h
		thirdparty/urdf/boost_replacement/shared_ptr.h
		thirdparty/urdf/boost_replacement/printf_console.cpp
		thirdparty/urdf/boost_replacement/printf_console.h
		thirdparty/urdf/boost_replacement/string_split.cpp
		thirdparty/urdf/boost_replacement/string_split.h
		)
	SET (URDFREADER_DEPENDENCIES
		rbdl
		)
ENDIF()

ADD_EXECUTABLE (rbdl_urdfreader_util rbdl_urdfreader_util.cc)

IF (RBDL_BUILD_STATIC)
        ADD_LIBRARY ( rbdl_urdfreader-static STATIC ${URDFREADER_SOURCES} )
        target_compile_definitions(rbdl_urdfreader-static PRIVATE ${URDFREADER_COMPILE_DEFINITIONS})

        SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES PREFIX "lib")
        SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES OUTPUT_NAME "rbdl_urdfreader")

        TARGET_LINK_LIBRARIES (rbdl_urdfreader-static
                rbdl-static
                )

        TARGET_LINK_LIBRARIES (rbdl_urdfreader_util
                rbdl_urdfreader-static
                rbdl-static
                )

        INSTALL (TARGETS rbdl_urdfreader-static rbdl_urdfreader_util
                EXPORT rbdl_urdfreader-static
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION lib
        )
ELSE (RBDL_BUILD_STATIC)
        ADD_LIBRARY ( rbdl_urdfreader SHARED ${URDFREADER_SOURCES} )
        target_compile_definitions(rbdl_urdfreader PRIVATE ${URDFREADER_COMPILE_DEFINITIONS})
        SET_TARGET_PROPERTIES ( rbdl_urdfreader PROPERTIES
                VERSION ${RBDL_VERSION}
                SOVERSION ${RBDL_SO_VERSION}
  )

        TARGET_LINK_LIBRARIES (rbdl_urdfreader
                ${URDFREADER_DEPENDENCIES}
                )

        TARGET_LINK_LIBRARIES (rbdl_urdfreader_util
                rbdl_urdfreader
                )

        INSTALL (TARGETS rbdl_urdfreader rbdl_urdfreader_util
                EXPORT rbdl_urdfreader
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
                LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                )
ENDIF (RBDL_BUILD_STATIC)

FILE ( GLOB headers
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )

INSTALL ( FILES ${headers}
        DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/rbdl/addons/urdfreader
        )
