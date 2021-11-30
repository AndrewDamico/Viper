# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "model_server: 1 messages, 2 services")

set(MSG_I_FLAGS "-Imodel_server:/home/andrew/viper/src/model_server/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(model_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_custom_target(_model_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "model_server" "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" ""
)

get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_custom_target(_model_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "model_server" "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" "std_msgs/Header:sensor_msgs/Image:std_msgs/Bool"
)

get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_custom_target(_model_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "model_server" "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" "std_msgs/String:model_server/InferenceResults"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(model_server
  "/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server
)

### Generating Services
_generate_srv_cpp(model_server
  "/home/andrew/viper/src/model_server/srv/ImageRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server
)
_generate_srv_cpp(model_server
  "/home/andrew/viper/src/model_server/srv/ModelRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server
)

### Generating Module File
_generate_module_cpp(model_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(model_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(model_server_generate_messages model_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_dependencies(model_server_generate_messages_cpp _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_cpp _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_cpp _model_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(model_server_gencpp)
add_dependencies(model_server_gencpp model_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS model_server_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(model_server
  "/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server
)

### Generating Services
_generate_srv_eus(model_server
  "/home/andrew/viper/src/model_server/srv/ImageRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server
)
_generate_srv_eus(model_server
  "/home/andrew/viper/src/model_server/srv/ModelRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server
)

### Generating Module File
_generate_module_eus(model_server
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(model_server_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(model_server_generate_messages model_server_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_dependencies(model_server_generate_messages_eus _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_eus _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_eus _model_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(model_server_geneus)
add_dependencies(model_server_geneus model_server_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS model_server_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(model_server
  "/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server
)

### Generating Services
_generate_srv_lisp(model_server
  "/home/andrew/viper/src/model_server/srv/ImageRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server
)
_generate_srv_lisp(model_server
  "/home/andrew/viper/src/model_server/srv/ModelRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server
)

### Generating Module File
_generate_module_lisp(model_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(model_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(model_server_generate_messages model_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_dependencies(model_server_generate_messages_lisp _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_lisp _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_lisp _model_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(model_server_genlisp)
add_dependencies(model_server_genlisp model_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS model_server_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(model_server
  "/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server
)

### Generating Services
_generate_srv_nodejs(model_server
  "/home/andrew/viper/src/model_server/srv/ImageRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server
)
_generate_srv_nodejs(model_server
  "/home/andrew/viper/src/model_server/srv/ModelRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server
)

### Generating Module File
_generate_module_nodejs(model_server
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(model_server_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(model_server_generate_messages model_server_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_dependencies(model_server_generate_messages_nodejs _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_nodejs _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_nodejs _model_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(model_server_gennodejs)
add_dependencies(model_server_gennodejs model_server_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS model_server_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(model_server
  "/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
)

### Generating Services
_generate_srv_py(model_server
  "/home/andrew/viper/src/model_server/srv/ImageRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
)
_generate_srv_py(model_server
  "/home/andrew/viper/src/model_server/srv/ModelRequest.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/home/andrew/viper/src/model_server/msg/InferenceResults.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
)

### Generating Module File
_generate_module_py(model_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(model_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(model_server_generate_messages model_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andrew/viper/src/model_server/msg/InferenceResults.msg" NAME_WE)
add_dependencies(model_server_generate_messages_py _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ImageRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_py _model_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andrew/viper/src/model_server/srv/ModelRequest.srv" NAME_WE)
add_dependencies(model_server_generate_messages_py _model_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(model_server_genpy)
add_dependencies(model_server_genpy model_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS model_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/model_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(model_server_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(model_server_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(model_server_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/model_server
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(model_server_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(model_server_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(model_server_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/model_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(model_server_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(model_server_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(model_server_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/model_server
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(model_server_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(model_server_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(model_server_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/model_server/.+/__init__.pyc?$"
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(model_server_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(model_server_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(model_server_generate_messages_py geometry_msgs_generate_messages_py)
endif()
