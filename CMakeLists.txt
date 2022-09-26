set(CMAKE_CXX_STANDARD 14)

set(SRCS nestplacer.cpp
		 include/nestplacer/export.h
		 include/nestplacer/nestplacer.h
		 )

__cc_find(Boost)

if(NOT WIN32)
	add_compile_options(-fPIC)
	add_definitions(-DHAVE_PTHREADS)
endif()

set(LIBS libnest2d clipper3r nlopt trimesh2 polygonLib)
if(BUILD_NESTPLACER_STATIC)
    __add_real_target(nestplacer lib SOURCE ${SRCS}
                                LIB ${LIBS}
                                DEF _NESTPLACER_DLL
                                INC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOOST_INCLUDE_DIRS}
                                INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
								OPENMP
								)
else()
    __add_real_target(nestplacer dll SOURCE ${SRCS}
                                LIB ${LIBS}
                                DEF _NESTPLACER_DLL
                                INC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOOST_INCLUDE_DIRS}
                                INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
								OPENMP
								)
endif()

if(CC_INSTALL_NESTPLACER)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ nestplacer)
endif()
