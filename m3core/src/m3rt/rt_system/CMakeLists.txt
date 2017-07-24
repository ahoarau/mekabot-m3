cmake_minimum_required(VERSION 2.8)
project(rt_system)
set(LIBNAME "rt_system")

if(VIRTUAL_MEKA)
message(STATUS "Disabled kernel sync (Virtual installation only)")

get_property(DEFS DIRECTORY
   PROPERTY COMPILE_DEFINITIONS)
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS ${DEFS} __NO_KERNEL_SYNC__
   )
endif()


set(LIBS ${LIBS} ${YAMLCPP_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread ${Boost_LIBRARIES}  m3base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../ ${YAMLCPP_INCLUDE_DIRS} ${M3RT_INCLUDE_DIR}  ${THREADS_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIRS})


set(ALL_SRCS
  rt_data_service.cpp
  rt_log_service.cpp
  rt_service.cpp
  rt_system.cpp
)
set(ALL_HDRS
  rt_data_service.h
  rt_log_service.h
  rt_service.h
  rt_system.h
)

# Swig
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include(${SWIG_USE_FILE})
include_directories(${PYTHON_INCLUDE_PATH})
set(CMAKE_SWIG_FLAGS -O -c++)

set(M3_SWIG_MODULE_NAME "m3rt_system")
set_source_files_properties(${M3_SWIG_MODULE_NAME}.i PROPERTIES CPLUSPLUS ON)
swig_add_module(${M3_SWIG_MODULE_NAME} python ${M3_SWIG_MODULE_NAME}.i ${ALL_SRCS})
swig_link_libraries(${M3_SWIG_MODULE_NAME} ${PYTHON_LIBRARIES} m3base rt)
set_target_properties(${SWIG_MODULE_${M3_SWIG_MODULE_NAME}_REAL_NAME} PROPERTIES LINKER_LANGUAGE CXX)
add_custom_target(${M3_SWIG_MODULE_NAME} ALL DEPENDS ${SWIG_MODULE_${M3_SWIG_MODULE_NAME}_REAL_NAME} ${ALL_SRCS})
add_custom_target(${LIBNAME}-swig ALL DEPENDS ${M3_SWIG_MODULE_NAME})
# End swig


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


## Installation
install ( TARGETS ${SWIG_MODULE_${M3_SWIG_MODULE_NAME}_REAL_NAME}
   LIBRARY
     DESTINATION ${PYTHON_SITE_DIR}/m3
     COMPONENT library
)

install ( FILES ${CMAKE_CURRENT_BINARY_DIR}/${M3_SWIG_MODULE_NAME}.py
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library
)

set(M3CORE_PYTHON_DIR ${PYTHON_SITE_DIR}/m3/ CACHE STRING "m3core python install dir")
