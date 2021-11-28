# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "inland_ir_cam: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iinland_ir_cam:/home/andrew/viper/src/inland_ir_cam/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(inland_ir_cam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_custom_target(_inland_ir_cam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inland_ir_cam" "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(inland_ir_cam
  "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inland_ir_cam
)

### Generating Services

### Generating Module File
_generate_module_cpp(inland_ir_cam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inland_ir_cam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(inland_ir_cam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(inland_ir_cam_generate_messages inland_ir_cam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_dependencies(inland_ir_cam_generate_messages_cpp _inland_ir_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inland_ir_cam_gencpp)
add_dependencies(inland_ir_cam_gencpp inland_ir_cam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inland_ir_cam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(inland_ir_cam
  "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inland_ir_cam
)

### Generating Services

### Generating Module File
_generate_module_eus(inland_ir_cam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inland_ir_cam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(inland_ir_cam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(inland_ir_cam_generate_messages inland_ir_cam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_dependencies(inland_ir_cam_generate_messages_eus _inland_ir_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inland_ir_cam_geneus)
add_dependencies(inland_ir_cam_geneus inland_ir_cam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inland_ir_cam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(inland_ir_cam
  "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inland_ir_cam
)

### Generating Services

### Generating Module File
_generate_module_lisp(inland_ir_cam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inland_ir_cam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(inland_ir_cam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(inland_ir_cam_generate_messages inland_ir_cam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_dependencies(inland_ir_cam_generate_messages_lisp _inland_ir_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inland_ir_cam_genlisp)
add_dependencies(inland_ir_cam_genlisp inland_ir_cam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inland_ir_cam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(inland_ir_cam
  "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inland_ir_cam
)

### Generating Services

### Generating Module File
_generate_module_nodejs(inland_ir_cam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inland_ir_cam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(inland_ir_cam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(inland_ir_cam_generate_messages inland_ir_cam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_dependencies(inland_ir_cam_generate_messages_nodejs _inland_ir_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inland_ir_cam_gennodejs)
add_dependencies(inland_ir_cam_gennodejs inland_ir_cam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inland_ir_cam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(inland_ir_cam
  "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inland_ir_cam
)

### Generating Services

### Generating Module File
_generate_module_py(inland_ir_cam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inland_ir_cam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(inland_ir_cam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(inland_ir_cam_generate_messages inland_ir_cam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/inland_ir_cam/msg/MotionVectors.msg" NAME_WE)
add_dependencies(inland_ir_cam_generate_messages_py _inland_ir_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inland_ir_cam_genpy)
add_dependencies(inland_ir_cam_genpy inland_ir_cam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inland_ir_cam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inland_ir_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inland_ir_cam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(inland_ir_cam_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inland_ir_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inland_ir_cam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(inland_ir_cam_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inland_ir_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inland_ir_cam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(inland_ir_cam_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inland_ir_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inland_ir_cam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(inland_ir_cam_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inland_ir_cam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inland_ir_cam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inland_ir_cam
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(inland_ir_cam_generate_messages_py std_msgs_generate_messages_py)
endif()
