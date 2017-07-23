
# deduce the libraries suffix from the options
set(FIND_M3ENS_LIB_SUFFIX "")

# find the M3ENS include directory
find_path(M3ENS_ROOT m3ens
        PATH_SUFFIXES include
                PATHS
                /usr/local/
                /usr)

set(M3ENS_ROOT ${M3ENS_ROOT}/m3ens/)

message("M3ens Root dir : ${M3ENS_ROOT}")
# find the requested modules
set(M3ENS_FOUND TRUE) # will be set to false if one of the required modules is not found
set(FIND_M3ENS_LIB_PATHS
                /usr/local
                /usr)
message("FIND_M3ens_COMPONENT : [${M3ens_FIND_COMPONENTS}]")
foreach(FIND_M3ENS_COMPONENT ${M3ens_FIND_COMPONENTS})
    message("-----------------------------")
    string(TOLOWER ${FIND_M3ENS_COMPONENT} FIND_M3ENS_COMPONENT_LOWER)
    set(FIND_M3ENS_COMPONENT_NAME m3ens${FIND_M3ENS_COMPONENT_LOWER}${FIND_M3ENS_LIB_SUFFIX})
    message("Looking for : ${FIND_M3ENS_COMPONENT_NAME}")
    set(M3ENS_INCLUDE_DIR ${M3ENS_INCLUDE_DIR} ${M3ENS_ROOT}${FIND_M3ENS_COMPONENT_LOWER})
    message("Include dir : ${M3ENS_INCLUDE_DIR}")
    find_library(M3ENS_${FIND_M3ENS_COMPONENT} NAMES ${FIND_M3ENS_COMPONENT_NAME} PATHS ${FIND_M3ENS_LIB_PATHS} PATH_SUFFIXES lib)
    message("Looking for : M3ENS_${FIND_M3ENS_COMPONENT}")
    set(M3ENS_LIBRARIES ${M3ENS_LIBRARIES} ${M3ENS_${FIND_M3ENS_COMPONENT}})

endforeach()
message("M3ENS Libraries : ${M3ENS_LIBRARIES}")




