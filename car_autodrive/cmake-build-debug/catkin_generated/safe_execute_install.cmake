execute_process(COMMAND "/home/wzy/catkin_ws/src/car_autodrive/cmake-build-debug/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/wzy/catkin_ws/src/car_autodrive/cmake-build-debug/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
