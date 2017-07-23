cmake_minimum_required(VERSION 2.8)
project(m3core-python)

add_subdirectory(scripts)

file(GLOB ${PROJECT_NAME}_m3
    "m3/*.py"
)

find_package(PythonLibs 2.7 REQUIRED)
find_package( PythonInterp REQUIRED )

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
set(M3CORE_PYTHON_DIR ${PYTHON_SITE_DIR} PARENT_SCOPE)
install (
	FILES ${${PROJECT_NAME}_m3} 
	PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE
	DESTINATION ${PYTHON_SITE_DIR}/m3
	COMPONENT library
)

## Hardly Cpackable, let's use the above (simple) way instead
#find_program(PYTHON "python")
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
#
#    add_custom_target(m3core_py ALL DEPENDS ${OUTPUT})

#    install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install)")
#endif()

