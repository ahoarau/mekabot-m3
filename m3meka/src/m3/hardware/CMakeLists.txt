cmake_minimum_required(VERSION 2.8)
project(hardware)
set(LIBNAME "m3hardware")

m3generate_proto(ProtoSources ProtoHeaders PySources)

SET(LIBS ${RTAI_LIBRARIES}  pthread ${Boost_LIBRARIES}  m3toolbox ${orocos_kdl_LIBRARIES} m3base)


set( ALL_SRCS
actuator.cpp
actuator_virtual.cpp
async_io.cpp
ctrl_simple.cpp
factory_proxy.cpp
joint.cpp
joint_virtual.cpp
joint_slave.cpp
joint_zlift_virtual.cpp
joint_zlift.cpp
loadx1.cpp
loadx6.cpp
loadx6_virtual.cpp
log_test.cpp
motor.cpp
pwr.cpp
pwr_virtual.cpp
robot_monitor.cpp
sensor.cpp
transmission.cpp
actuator_ec.cpp
joint_zlift_shm.cpp
led_matrix_ec.cpp
led_matrix_ec_shm.cpp
ledx2_ec.cpp
ledx2xn_ec.cpp
ledx2xn_ec_shm.cpp
loadx1_ec.cpp
loadx6_ec.cpp
pwr_ec.cpp
tactile_pps22_ec.cpp
)

set( ALL_HDRS
actuator_ec.h
actuator.h
actuator_virtual.h
async_io.h
ctrl_simple.h
joint.h
joint_virtual.h
joint_slave.h
joint_zlift.h
joint_zlift_virtual.h
joint_zlift_shm.h
joint_zlift_shm_sds.h
led_matrix_ec.h
led_matrix_ec_shm.h
led_matrix_ec_shm_sds.h
ledx2_ec.h
ledx2xn_ec.h
ledx2xn_ec_shm.h
ledx2xn_ec_shm_sds.h
loadx1_ec.h
loadx1.h
loadx6_ec.h
loadx6.h
loadx6_virtual.h
log_test.h
m3ec_def_meka.h
m3ec_pdo_v0_def.h
m3ec_pdo_v1_def.h
m3ec_pdo_v2_def.h
m3ec_pdo_v3_def.h
m3ec_pdo_v4_def.h
m3meka_ec_def.h
motor.h
pwr_ec.h
pwr.h
pwr_virtual.h
robot_monitor.h
sensor.h
tactile_pps22_ec.h
transmission.h

)


list(APPEND ALL_SRCS ${ProtoSources})

list(APPEND ALL_HDRS
${ProtoHeaders}
${ProtoFiles}
)

## Building
add_library(${LIBNAME} SHARED ${ALL_SRCS})
target_link_libraries(${LIBNAME} ${LIBS})
#set_target_properties(${LIBNAME} PROPERTIES COMPILE_FLAGS "-I${CMAKE_BINARY_DIR}/src")

## Installing
install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${ALL_HDRS} DESTINATION include/m3/${PROJECT_NAME})

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


install ( FILES ${PySources}
   DESTINATION ${PYTHON_SITE_DIR}/m3
   COMPONENT library
)
