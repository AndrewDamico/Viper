execute_process(COMMAND "/home/andrew/viper/build/scene_segmentation_module/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/andrew/viper/build/scene_segmentation_module/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
