cmake_minimum_required(VERSION 2.5)
project(meka_cmake_modules)

# Overwrite option
option(OVERWRITE "Overwrite existing files in ${CMAKE_ROOT}/Modules" OFF)
# Printing a warning
if(${OVERWRITE})
	message("-- OVERWRITE Option is ON")
endif()


# Searching for all the modules
file(GLOB all_find_modules "${CMAKE_CURRENT_SOURCE_DIR}/Modules/*.cmake")

## Printing
#foreach(find_module ${all_find_modules})
#	message("-- Cmake module found : ${find_module}")
#endforeach()

# Installing them in /usr/local/lib/cmake/xxx/Findxxx.cmake
# According to http://www.cmake.org/cmake/help/v2.8.8/cmake.html#command:find_package

foreach(find_module ${all_find_modules})
	get_filename_component(Findname ${find_module} NAME_WE )
	get_filename_component(Findname_s ${find_module} NAME )
	string(REPLACE "Find" "" name ${Findname})
	
	if(EXISTS "${CMAKE_ROOT}/Modules/${Findname_s}")
		message( "${Findname_s} already exists in ${CMAKE_ROOT}/Modules !")
		if(${OVERWRITE})
			message("-- ${Findname_s} will be installed in /usr/local/lib/cmake/${name}/ (OVERWRITTING)")
			install(FILES ${find_module} DESTINATION ${CMAKE_ROOT}/Modules)
		else(${OVERWRITE})
			message("Use -DOVERWRITE=1 if you want to overwrite those modules (use at your own risks")
		endif(${OVERWRITE})
	else()
		message("-- ${Findname_s} will be installed in /usr/local/lib/cmake/${name}/")
		install(FILES ${find_module} DESTINATION ${CMAKE_ROOT}/Modules)
	endif()
	message("")
endforeach()

