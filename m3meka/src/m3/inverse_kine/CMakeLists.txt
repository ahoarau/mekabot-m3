cmake_minimum_required(VERSION 2.8)
project(inverse_kine)

file(GLOB ALL_HDRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} [^.]*_ikfast.h)

# Swig
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp REQUIRED) 
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_SWIG_FLAGS "")

## Processing all the files
file(GLOB ALL_HDRS_FOR_SWIG RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} [^.]*_for_swig.h)

set(ALL_PY)
set(ALL_MODULES)
set(ALL_HDRS)
foreach(f ${ALL_HDRS_FOR_SWIG})
get_filename_component(varik ${f} NAME_WE )
string(REPLACE "_for_swig" "" varik_base ${varik})
set(swig_module_tmp ${varik_base})
set(swig_bridge_file_tmp "${swig_module_tmp}.i")
set(swig_wrap_tmp "${swig_module_tmp}.cpp")
set(swig_target_tmp "_${varik_base}_")
#message("Processing ${f} -> ${varik} -> ${varik_base} -> ${swig_bridge_file_tmp} -> ${swig_wrap_tmp} -> ${swig_module_tmp}")

SET_SOURCE_FILES_PROPERTIES(${swig_bridge_file_tmp} PROPERTIES CPLUSPLUS ON)
SWIG_ADD_MODULE(${swig_module_tmp} python ${swig_bridge_file_tmp} )
SWIG_LINK_LIBRARIES(${swig_module_tmp} ${PYTHON_LIBRARIES})
#set_target_properties(_m3rt_system PROPERTIES LINKER_LANGUAGE CXX)

add_custom_target(${swig_target_tmp}  ALL DEPENDS ${f})



list(APPEND ALL_MODULES ${SWIG_MODULE_${swig_module_tmp}_REAL_NAME})
list(APPEND ALL_PY ${swig_module_tmp})
list(APPEND ALL_HDRS ${f})
endforeach()
#message("All swig-compiled tar : ${ALL_TAR}")
#message("All swig-compiled modules : ${ALL_MODULES}")
#message("All swig headers : ${ALL_HDRS}")

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
install ( TARGETS ${ALL_MODULES}
   LIBRARY 
     DESTINATION ${PYTHON_SITE_DIR}/m3 
     COMPONENT library 
) 
foreach(swig_py_file ${ALL_PY})
install ( FILES ${CMAKE_CURRENT_BINARY_DIR}/${swig_py_file}.py
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library 
) 
endforeach()


# End Swig

## Python stuff
#find_program(PYTHON "python")
##set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


#if (PYTHON)
#    set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
#    set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
#    set(DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/module/__init__.py")
#    set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build/timestamp")

#    configure_file(${SETUP_PY_IN} ${SETUP_PY})

#    add_custom_command(OUTPUT ${OUTPUT}
#                       COMMAND ${PYTHON} ${SETUP_PY} build
#                       COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
#                       DEPENDS ${DEPS})

 #   add_custom_target(inverse_kin_py ALL DEPENDS ${OUTPUT})

#    install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
#endif()
## End Python Stuff
install(FILES ${ALL_HDRS} DESTINATION  include/m3/${PROJECT_NAME})

