

if(VIRTUAL_MEKA)
	set(VIRTUAL 1)
else()
	set(VIRTUAL 0)
endif()

configure_file(uninstall_m3.in ${CMAKE_CURRENT_BINARY_DIR}/uninstall_m3)
configure_file(m3rt_server_run.in ${CMAKE_CURRENT_BINARY_DIR}/m3rt_server_run)

set(M3RTSCRIPTS
disable_ros
lsec
m3rt_insmods
m3rt_rmmods
m3rt_ethercat_check
m3rt_rotate_logs
m3rt_server_debug
m3rt_server_kill
m3rt_timestats_run
disable_kdl_ros
${CMAKE_CURRENT_BINARY_DIR}/uninstall_m3
${CMAKE_CURRENT_BINARY_DIR}/m3rt_server_run
)
install (
	FILES ${M3RTSCRIPTS} 
	PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE
	DESTINATION bin
)


