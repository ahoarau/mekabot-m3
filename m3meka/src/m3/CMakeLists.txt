
set(SUBS
  chains
  hardware
  inverse_kine
  robot_ctrl
  robots
  toolbox
  vehicles
)

if(RTAI_FOUND)
  list(APPEND SUBS
  shared_mem)
endif()

foreach(SUB ${SUBS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/${SUB})
  set(PROTOBUF_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/${SUB})
endforeach()

foreach(SUB ${SUBS})
  add_subdirectory(${SUB})
endforeach()
