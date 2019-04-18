cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.5.3")
project("SolARModuleOpenGL")
set (SOURCES 
	src/SolARModuleOpengl.cpp
	src/SolAR3DPointsViewerOpengl.cpp
	src/SolARSinkPoseTextureBufferOpengl.cpp
	src/glcamera/gl_camera.cpp
)

set (HEADERS
	interfaces/SolAROpenglAPI.h
	interfaces/SolARModuleOpengl_traits.h
	interfaces/SolAR3DPointsViewerOpengl.h
	interfaces/SolARSinkPoseTextureBufferOpengl.h
	src/glcamera/common.hpp
	src/glcamera/gl_camera.hpp
	src/glcamera/math.hpp
	src/glcamera/matrix.hpp
	src/glcamera/matrix_fixed.hpp
	src/glcamera/rigid_motion.hpp
	src/glcamera/trackball.hpp
	src/glcamera/vector.hpp
	src/glcamera/vector_fixed.hpp
)

find_package(OpenGL REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIR} )
#message(STATUS, "############# ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}")
set (LINKER_OPTIONS_RELEASE "optimized  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}")
set (LINKER_OPTIONS_DEBUG "debug  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}")


include_directories(
	src/
)

install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/xpcf_SolARModuleOpenGL_registry.xml" DESTINATION $ENV{BCOMDEVROOT}/.xpcf/SolAR/	)

##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


