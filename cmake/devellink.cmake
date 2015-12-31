
macro(generate_symlink_devel)
    # Usage : generate_symlink_devel link path
    # This creates a symlink in your current source dir, so that normal python finds catkin generated classes in devel/

    set(link "${CMAKE_CURRENT_SOURCE_DIR}/src/pyros/cfg")
    set(path "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg")

    if(NOT EXISTS "${link}" AND EXISTS "${path}")
        message(STATUS "linking catkin generated classes from ${path} to source space in ${link}")
        execute_process(
          # Note : return code is broken in 2.8 (trusty version) https://cmake.org/Bug/view.php?id=14928
          COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" "${path}" "${link}"
        )
    endif()
endmacro()