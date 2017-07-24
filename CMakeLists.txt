cmake_minimum_required(VERSION 2.8)
project(mekabot)

## Mekabot Version
set(MAJOR_VERSION "2")
set(MINOR_VERSION "0")
set(PATCH_VERSION "0")
set(M3_VERSION
  ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/meka-cmake-modules/Modules)

set(KERNEL_NAME "" CACHE STRING "The rtai patched kernel to use. -DKERNEL_NAME=$(uname -r)")

if(KERNEL_NAME)
  set(CMAKE_SYSTEM_VERSION ${KERNEL_NAME})
  message("-- Custom kernel : ${KERNEL_NAME}")
endif()

find_package(RTAI QUIET)
if(RTAI_FOUND)
  message(INFO "Building with RTAI")
  add_definitions(-D__RTAI__)
  include_directories(${RTAI_INCLUDE_DIR})
  link_libraries(${RTAI_LIBRARY})
else()
  message(WARNING "RTAI not detected, use for simulation only.")
endif()

find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
find_package(YamlCpp REQUIRED)
find_package(Threads REQUIRED)
find_package(SWIG REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(M3RT_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/m3core/src
  ${CMAKE_CURRENT_BINARY_DIR}/m3core/src
)

set(M3_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/m3meka/src
  ${CMAKE_CURRENT_BINARY_DIR}/m3meka/src
)

set(PROTOBUF_IMPORT_DIRS
  ${CMAKE_CURRENT_BINARY_DIR}/m3core/src/m3rt/base
)

function(m3generate_proto srcs hdrs py_srcs)

  file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

  foreach(ProtoFile ${ProtoFiles})
  	get_filename_component(proto ${ProtoFile} NAME)
  	configure_file(${ProtoFile} ${CMAKE_CURRENT_BINARY_DIR}/${proto} COPYONLY)
  endforeach()

  protobuf_generate_python(PySources ${ProtoFiles})
  add_custom_target(${proto}_python_gen ALL DEPENDS ${PySources})
  protobuf_generate_cpp(ProtoSources ProtoHeaders ${ProtoFiles})
  add_custom_target(${proto}_cpp_gen ALL DEPENDS ${ProtoSources})

  set(${srcs} ${ProtoSources} PARENT_SCOPE)
  set(${hdrs} ${ProtoHeaders} PARENT_SCOPE)
  set(${py_srcs} ${PySources} PARENT_SCOPE)
endfunction()

include_directories(${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR} ${PROTOBUF_IMPORT_DIRS})

configure_file(setup.bash.in ${CMAKE_CURRENT_BINARY_DIR}/setup.bash)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.bash DESTINATION share)


add_subdirectory(holomni_pcv)

add_subdirectory(m3core)
add_subdirectory(m3meka)
