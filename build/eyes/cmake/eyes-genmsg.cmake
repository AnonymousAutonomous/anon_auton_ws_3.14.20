# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "eyes: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ieyes:/home/anonymous3/anon_auton_ws/src/eyes/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(eyes_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_custom_target(_eyes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eyes" "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" ""
)

get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_custom_target(_eyes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eyes" "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" ""
)

get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_custom_target(_eyes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eyes" "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes
)
_generate_msg_cpp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes
)
_generate_msg_cpp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes
)

### Generating Services

### Generating Module File
_generate_module_cpp(eyes
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(eyes_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(eyes_generate_messages eyes_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_dependencies(eyes_generate_messages_cpp _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_dependencies(eyes_generate_messages_cpp _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_dependencies(eyes_generate_messages_cpp _eyes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eyes_gencpp)
add_dependencies(eyes_gencpp eyes_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eyes_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes
)
_generate_msg_eus(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes
)
_generate_msg_eus(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes
)

### Generating Services

### Generating Module File
_generate_module_eus(eyes
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(eyes_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(eyes_generate_messages eyes_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_dependencies(eyes_generate_messages_eus _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_dependencies(eyes_generate_messages_eus _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_dependencies(eyes_generate_messages_eus _eyes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eyes_geneus)
add_dependencies(eyes_geneus eyes_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eyes_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes
)
_generate_msg_lisp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes
)
_generate_msg_lisp(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes
)

### Generating Services

### Generating Module File
_generate_module_lisp(eyes
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(eyes_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(eyes_generate_messages eyes_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_dependencies(eyes_generate_messages_lisp _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_dependencies(eyes_generate_messages_lisp _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_dependencies(eyes_generate_messages_lisp _eyes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eyes_genlisp)
add_dependencies(eyes_genlisp eyes_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eyes_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes
)
_generate_msg_nodejs(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes
)
_generate_msg_nodejs(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes
)

### Generating Services

### Generating Module File
_generate_module_nodejs(eyes
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(eyes_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(eyes_generate_messages eyes_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_dependencies(eyes_generate_messages_nodejs _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_dependencies(eyes_generate_messages_nodejs _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_dependencies(eyes_generate_messages_nodejs _eyes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eyes_gennodejs)
add_dependencies(eyes_gennodejs eyes_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eyes_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes
)
_generate_msg_py(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes
)
_generate_msg_py(eyes
  "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes
)

### Generating Services

### Generating Module File
_generate_module_py(eyes
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(eyes_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(eyes_generate_messages eyes_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Choreo.msg" NAME_WE)
add_dependencies(eyes_generate_messages_py _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Custom.msg" NAME_WE)
add_dependencies(eyes_generate_messages_py _eyes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anonymous3/anon_auton_ws/src/eyes/msg/Autonomous.msg" NAME_WE)
add_dependencies(eyes_generate_messages_py _eyes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eyes_genpy)
add_dependencies(eyes_genpy eyes_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eyes_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eyes
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(eyes_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(eyes_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eyes
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(eyes_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(eyes_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eyes
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(eyes_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(eyes_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eyes
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(eyes_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(eyes_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eyes
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(eyes_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(eyes_generate_messages_py std_msgs_generate_messages_py)
endif()
