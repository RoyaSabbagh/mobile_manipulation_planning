# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pam: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ipam:/home/roya/catkin_ws/src/pam_manipulation_planning/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" ""
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" ""
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" ""
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" ""
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" ""
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" "pam/state"
)

get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_custom_target(_pam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pam" "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg"
  "${MSG_I_FLAGS}"
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)
_generate_msg_cpp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
)

### Generating Services

### Generating Module File
_generate_module_cpp(pam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pam_generate_messages pam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_dependencies(pam_generate_messages_cpp _pam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pam_gencpp)
add_dependencies(pam_gencpp pam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg"
  "${MSG_I_FLAGS}"
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)
_generate_msg_eus(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
)

### Generating Services

### Generating Module File
_generate_module_eus(pam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pam_generate_messages pam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_dependencies(pam_generate_messages_eus _pam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pam_geneus)
add_dependencies(pam_geneus pam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg"
  "${MSG_I_FLAGS}"
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)
_generate_msg_lisp(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
)

### Generating Services

### Generating Module File
_generate_module_lisp(pam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pam_generate_messages pam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_dependencies(pam_generate_messages_lisp _pam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pam_genlisp)
add_dependencies(pam_genlisp pam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg"
  "${MSG_I_FLAGS}"
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)
_generate_msg_nodejs(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pam_generate_messages pam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_dependencies(pam_generate_messages_nodejs _pam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pam_gennodejs)
add_dependencies(pam_gennodejs pam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg"
  "${MSG_I_FLAGS}"
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)
_generate_msg_py(pam
  "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
)

### Generating Services

### Generating Module File
_generate_module_py(pam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pam_generate_messages pam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/state.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/controlInput.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/JoyStick_cmd.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/force_reading.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Gripper_mode.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/feedback.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/roya/catkin_ws/src/pam_manipulation_planning/msg/Roomba_cmd_vel.msg" NAME_WE)
add_dependencies(pam_generate_messages_py _pam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pam_genpy)
add_dependencies(pam_genpy pam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pam_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pam_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pam_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pam_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pam/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pam_generate_messages_py std_msgs_generate_messages_py)
endif()
