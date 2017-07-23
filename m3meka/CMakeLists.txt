project(m3meka)

# Include dirs to look for
include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(src/m3)
add_subdirectory(python)
add_subdirectory(scripts)
