macro(catkin_pip_install_requirements devel_path)
    # devel_path should be set to ${CATKIN_DEVEL_PREFIX}/${CATKING_GLOBAL_PYTHON_DESTINATION
    set(PIP_INSTALL_REQ_DEVEL "/usr/bin/pip install -r requirements.txt -t ${devel_path}")

    message(STATUS "    ... Installing Pip requirements from ${CMAKE_CURRENT_SOURCE_DIR} in ${devel_path}.")

    execute_process(
      COMMAND  ${PIP_INSTALL_REQ_DEVEL}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE PIP_RESULT
      OUTPUT_VARIABLE PIP_VARIABLE)

    message(STATUS "    ... Done.. [${PIP_RESULT}]: ${PIP_VARIABLE}")

endmacro()
