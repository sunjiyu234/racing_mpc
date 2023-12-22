# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ltv_mpc: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iltv_mpc:/home/sun234/racing_work/src/ltv_mpc/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ltv_mpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_custom_target(_ltv_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ltv_mpc" "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" ""
)

get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_custom_target(_ltv_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ltv_mpc" "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" "ltv_mpc/sample"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltv_mpc
)
_generate_msg_cpp(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg"
  "${MSG_I_FLAGS}"
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltv_mpc
)

### Generating Services

### Generating Module File
_generate_module_cpp(ltv_mpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltv_mpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ltv_mpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ltv_mpc_generate_messages ltv_mpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_cpp _ltv_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_cpp _ltv_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltv_mpc_gencpp)
add_dependencies(ltv_mpc_gencpp ltv_mpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltv_mpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltv_mpc
)
_generate_msg_eus(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg"
  "${MSG_I_FLAGS}"
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltv_mpc
)

### Generating Services

### Generating Module File
_generate_module_eus(ltv_mpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltv_mpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ltv_mpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ltv_mpc_generate_messages ltv_mpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_eus _ltv_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_eus _ltv_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltv_mpc_geneus)
add_dependencies(ltv_mpc_geneus ltv_mpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltv_mpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltv_mpc
)
_generate_msg_lisp(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg"
  "${MSG_I_FLAGS}"
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltv_mpc
)

### Generating Services

### Generating Module File
_generate_module_lisp(ltv_mpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltv_mpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ltv_mpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ltv_mpc_generate_messages ltv_mpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_lisp _ltv_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_lisp _ltv_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltv_mpc_genlisp)
add_dependencies(ltv_mpc_genlisp ltv_mpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltv_mpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltv_mpc
)
_generate_msg_nodejs(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg"
  "${MSG_I_FLAGS}"
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltv_mpc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ltv_mpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltv_mpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ltv_mpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ltv_mpc_generate_messages ltv_mpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_nodejs _ltv_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_nodejs _ltv_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltv_mpc_gennodejs)
add_dependencies(ltv_mpc_gennodejs ltv_mpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltv_mpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc
)
_generate_msg_py(ltv_mpc
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg"
  "${MSG_I_FLAGS}"
  "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc
)

### Generating Services

### Generating Module File
_generate_module_py(ltv_mpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ltv_mpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ltv_mpc_generate_messages ltv_mpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_py _ltv_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sun234/racing_work/src/ltv_mpc/msg/sample_lst.msg" NAME_WE)
add_dependencies(ltv_mpc_generate_messages_py _ltv_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltv_mpc_genpy)
add_dependencies(ltv_mpc_genpy ltv_mpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltv_mpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltv_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltv_mpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ltv_mpc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltv_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltv_mpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ltv_mpc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltv_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltv_mpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ltv_mpc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltv_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltv_mpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ltv_mpc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltv_mpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ltv_mpc_generate_messages_py std_msgs_generate_messages_py)
endif()
