macro(catkin_pip_install)
    set(NPM_UPDATE_BIN "/usr/bin/npm")
    set(NPM_UPDATE "update")

    message(STATUS "    ... Installing Pip requirements")
    message(STATUS "    ... CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

    # Not sure whether there is a way to make it custom target
    safe_execute_process(
      COMMAND  ${NPM_UPDATE_BIN} ${NPM_UPDATE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE NPM_RESULT
      OUTPUT_VARIABLE NPM_VARIABLE)

    message(STATUS "    ... Done.. [${NPM_RESULT}]: ${NPM_VARIABLE}")

endmacro()