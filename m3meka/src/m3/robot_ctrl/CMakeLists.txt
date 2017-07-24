cmake_minimum_required(VERSION 2.8)
project(robot_ctrl)
set(LIBNAME "m3robot_ctrl")


m3generate_proto(ProtoSources ProtoHeaders PySources)

SET(LIBS ${RTAI_LIBRARIES}  pthread ${Boost_LIBRARIES}  ${PROTOBUF_LIBRARY} m3robots m3chains m3hardware m3toolbox m3base)


set( ALL_SRCS
factory_proxy.cpp
head_s2csp_ctrl.cpp

)
set( ALL_HDRS
head_s2csp_ctrl.h
)
list(APPEND ALL_SRCS ${ProtoSources})


#file(GLOB ProtoFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.proto )
list(APPEND ALL_HDRS
${ProtoHeaders}
${ProtoFiles}
)


## Building
add_library(${LIBNAME} SHARED ${ALL_SRCS})
target_link_libraries(${LIBNAME} ${LIBS})
set_target_properties(${LIBNAME} PROPERTIES COMPILE_FLAGS "-I${CMAKE_BINARY_DIR}/src")

## Installing
install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${ALL_HDRS} DESTINATION include/m3/${PROJECT_NAME})


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
