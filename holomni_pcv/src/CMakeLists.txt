
include_directories(${YAMLCPP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/matrix)

file(GLOB sources
"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/matrix/*.cpp")

file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB matrix_headers ${CMAKE_CURRENT_SOURCE_DIR}/matrix/*.h)

foreach(header ${headers})
  get_filename_component(header_fn ${header} NAME)
  configure_file(${header} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/${header_fn} COPYONLY)
endforeach()

foreach(header ${matrix_headers})
  get_filename_component(header_fn ${header} NAME)
  configure_file(${header} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/matrix/${header_fn} COPYONLY)
endforeach()

add_library(${LIBNAME} SHARED ${sources})
target_link_libraries(${LIBNAME} ${YAMLCPP_LIBRARIES})

install(TARGETS ${LIBNAME} DESTINATION lib COMPONENT library)
install(FILES ${headers} DESTINATION include/${PROJECT_NAME})
install(FILES ${matrix_headers} DESTINATION include/${PROJECT_NAME}/matrix)
