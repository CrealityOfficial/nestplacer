set(CMAKE_CXX_STANDARD 14)

__files_group(${CMAKE_CURRENT_SOURCE_DIR}/nestplacer/ SRCS)
__files_group(${CMAKE_CURRENT_SOURCE_DIR}/impl/ IMPLS)

list(APPEND SRCS ${IMPLS})
__cc_find(trimesh2)
__assert_target(trimesh2)

__cc_find(libnest2d)
__cc_find(clipper3r)
__cc_find(nlopt_static)

__assert_target(libnest2d)
__assert_target(clipper3r)
__assert_target(nlopt_static)

set(LIBS libnest2d clipper3r nlopt_static)
set(ILIBS trimesh2)

__add_common_library(nestplacer OPENMP)

if(CC_INSTALL_NESTPLACER)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/nestplacer nestplacer)
endif()
