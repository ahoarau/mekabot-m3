


add_subdirectory(base)
add_subdirectory(rt_system)

if(RTAI_FOUND)
  add_subdirectory(ethercat)
endif()
