FIND_PACKAGE(OpenGL)
FIND_PACKAGE(GLUT)

IF ( NOT GLUT_FOUND )
	IF ( MSVC )
		SET ( GLUT_FOUND 1 )
		SET ( GLUT_INCLUDE_DIR ${Assimp_SOURCE_DIR}/samples/glut/ )
		SET ( GLUT_LIBRARIES ${Assimp_SOURCE_DIR}/samples/glut/glut32.lib )
	ELSE ( MSVC )
		MESSAGE( WARNING "Please install glut." )
	ENDIF ( MSVC )
ENDIF ( NOT GLUT_FOUND )

find_package(PkgConfig REQUIRED)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (NOT ASSIMP_FOUND)
  message(WARNING "ASsimp not found, not building synthetic views")
endif()
if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
  set(IS_ASSIMP3 1)
  add_definitions(-DIS_ASSIMP3=1)
  message(STATUS "Found assimp v3")
else()
  add_definitions(-DIS_ASSIMP3=0)
  message(STATUS "Found assimp v2")
endif()

include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(SYSTEM ${GLUT_INCLUDE_DIR}
                           ${OpenCV_INCLUDE_DIR}
                           ${OPENGL_INCLUDE_DIR}
)

ADD_EXECUTABLE( assimp_simpleogl display.cpp
                                 renderer.cpp
                                 Model.cpp
)

SET_PROPERTY(TARGET assimp_simpleogl PROPERTY DEBUG_POSTFIX ${ASSIMP_DEBUG_POSTFIX})

TARGET_LINK_LIBRARIES( assimp_simpleogl assimp
                                        ${OpenCV_LIBRARIES}
                                        ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} IL ILU ILUT freeimage)
SET_TARGET_PROPERTIES(assimp_simpleogl PROPERTIES
                                       OUTPUT_NAME assimp_simpleogl
)
