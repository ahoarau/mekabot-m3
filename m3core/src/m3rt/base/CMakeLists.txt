cmake_minimum_required(VERSION 3.1)
project(base)
set(SUBPROJECT_INSTALL_DIR_NAME m3rt)
set(SUBPROJECT_INSTALL_NAME ${PROJECT_NAME})
set(LIBNAME "m3base")

m3generate_proto(ProtoSources ProtoHeaders PySources)

set(LIBS ${LIBS} ${YAMLCPP_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread ${Boost_LIBRARIES} )

## Get all the files sources and headers
set( all_srcs
	component_async.cpp
	component_ec.cpp
	component_factory.cpp
	component_shm.cpp
	component.cpp
	simple_server.cpp
	toolbox.cpp
)

set(all_hdrs
	component_async.h
	component_ec.h
	component_factory.h
	component.h
	m3ec_def.h
	m3rt_def.h
	simple_server.h
	toolbox.h
	component_shm.h
)

list(APPEND all_srcs ${ProtoSources})

list(APPEND all_hdrs ${ProtoHeaders} ${ProtoFiles})

add_library(${LIBNAME} SHARED ${all_srcs})
target_link_libraries(${LIBNAME} ${LIBS})

install(TARGETS ${LIBNAME} DESTINATION lib COMPONENT library)
install(FILES ${all_hdrs} DESTINATION include/${SUBPROJECT_INSTALL_DIR_NAME}/${SUBPROJECT_INSTALL_NAME})

execute_process (
   COMMAND ${PYTHON_EXECUTABLE} -c
   	"import site, sys; sys.stdout.write(site.PREFIXES[-1])"
   OUTPUT_VARIABLE PYTHON_PREFIX
)
file ( TO_CMAKE_PATH "${PYTHON_PREFIX}" PYTHON_PREFIX )
execute_process (
   COMMAND ${PYTHON_EXECUTABLE} -c
   	"import site, sys; sys.stdout.write(site.getsitepackages()[-1])"
   OUTPUT_VARIABLE PYTHON_SITE_DIR
)
file ( TO_CMAKE_PATH "${PYTHON_SITE_DIR}" PYTHON_SITE_DIR )
string ( REGEX REPLACE "^${PYTHON_PREFIX}/" ""
   PYTHON_SITE_DIR "${PYTHON_SITE_DIR}"
)


install ( FILES ${PySources}
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library
)
