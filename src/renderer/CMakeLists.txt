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
  set(EXTRA_SOURCES)
  set(EXTRA_LIBRARIES assimp)
else()
  add_definitions(-DIS_ASSIMP3=0)
  message(STATUS "Found assimp v2")
  include_directories(./assimp/include)
  aux_source_directory(./assimp/contrib/clipper EXTRA_SOURCES_clipper)
  aux_source_directory(./assimp/contrib/ConvertUTF EXTRA_SOURCES_ConvertUTF)
  aux_source_directory(./assimp/contrib/irrXML EXTRA_SOURCES_irrXML)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/common EXTRA_SOURCES_poly2tri)
  aux_source_directory(./assimp/contrib/poly2tri/poly2tri/sweep EXTRA_SOURCES_poly2tri_sweep)
  aux_source_directory(./assimp/contrib/unzip EXTRA_SOURCES_unzip)
  aux_source_directory(./assimp/contrib/zlib EXTRA_SOURCES_zlib)
  aux_source_directory(./assimp/code EXTRA_SOURCES)
  set(EXTRA_SOURCES ${EXTRA_SOURCES} ${EXTRA_SOURCES_clipper} ${EXTRA_SOURCES_ConvertUTF} ${EXTRA_SOURCES_irrXML} ${EXTRA_SOURCES_poly2tri} ${EXTRA_SOURCES_poly2tri_sweep} ${EXTRA_SOURCES_unzip} ${EXTRA_SOURCES_zlib})
  message(INFO ${EXTRA_SOURCES})
  set(EXTRA_LIBRARIES)
endif()

include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(SYSTEM ${GLUT_INCLUDE_DIR}
                           ${OpenCV_INCLUDE_DIR}
                           ${OPENGL_INCLUDE_DIR}
)

ADD_EXECUTABLE(view_generator display.cpp
                              renderer.cpp
                              Model.cpp
                              ${EXTRA_SOURCES}
)

SET_PROPERTY(TARGET view_generator PROPERTY DEBUG_POSTFIX ${ASSIMP_DEBUG_POSTFIX})

TARGET_LINK_LIBRARIES(view_generator ${EXTRA_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
                                     ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} IL ILU ILUT freeimage)
SET_TARGET_PROPERTIES(view_generator PROPERTIES
                                     OUTPUT_NAME view_generator
)
