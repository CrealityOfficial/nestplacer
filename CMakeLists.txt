__files_group(${CMAKE_CURRENT_SOURCE_DIR}/ SRC)

if(NOT WIN32)
	add_compile_options(-fPIC)
	add_definitions(-DHAVE_PTHREADS)
endif()
set(LIBS libnest2d clipper3r nlopt trimesh2)

__add_real_target(nestplacer dll SOURCE ${SRC} 
								LIB ${LIBS}
								DEF _NESTPLACER_DLL
								INC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOOST_INCLUDE_DIRS}
								INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(APPLE)
    __add_openmp_lib(nestplacer)
endif()