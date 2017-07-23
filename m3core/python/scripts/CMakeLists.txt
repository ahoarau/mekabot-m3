

cmake_minimum_required(VERSION 2.8)
project(m3core-python-scripts)

file(GLOB ${PROJECT_NAME}_scripts
    "m3rt/*.py"
    "dev/*.py"
)

configure_file(m3rt/m3rt_check_server.py.in ${CMAKE_CURRENT_BINARY_DIR}/m3rt/m3rt_check_server.py)
list(APPEND ${PROJECT_NAME}_scripts ${CMAKE_CURRENT_BINARY_DIR}/m3rt/m3rt_check_server.py)

install (
	FILES ${${PROJECT_NAME}_scripts} 
	PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE
	DESTINATION bin
)




