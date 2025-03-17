execute_process(COMMAND "/home/leon/shared_ws/cecilia_ws/src/optitrack/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/leon/shared_ws/cecilia_ws/src/optitrack/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
