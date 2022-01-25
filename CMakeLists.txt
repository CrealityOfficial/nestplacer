set(SRCS nestplacer.cpp
		 include/nestplacer/export.h
		 include/nestplacer/nestplacer.h
		 )

if(NOT WIN32)
	add_compile_options(-fPIC)
	add_definitions(-DHAVE_PTHREADS)
endif()
set(LIBS libnest2d clipper3r nlopt trimesh2)

__add_real_target(nestplacer dll SOURCE ${SRCS} 
								LIB ${LIBS}
								DEF _NESTPLACER_DLL
								INC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOOST_INCLUDE_DIRS}
								INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(APPLE)
    __add_openmp_lib(nestplacer)
endif()