find_package( PythonInterp REQUIRED )

# This sets a virtual environment in the catkin devel space
  get_filename_component( activate_full "${CATKIN_DEVEL_PREFIX}/bin/activate"
    ABSOLUTE )
  if( NOT EXISTS "${activate_full}" )
    find_program( VIRTUALENV NAMES virtualenv virtualenv.py )
    if( VIRTUALENV )
      set( virtualenv_script "${VIRTUALENV}" )
    else( VIRTUALENV )
      message( SEND_ERROR "virtualenv command not found. Make sure you have installed the python-virtualenv package.")
    endif( VIRTUALENV )
    # Create the virtual environment.
    execute_process( COMMAND "${PYTHON_EXECUTABLE}"
      "${virtualenv_script}"
      "--python=${PYTHON_EXECUTABLE}"
      "--system-site-packages"
      "${CATKIN_DEVEL_PREFIX}"
    RESULT_VARIABLE failed
    ERROR_VARIABLE error )
    if( failed )
      message( SEND_ERROR ${error} )
    endif()
  else( NOT EXISTS "${activate_full}" )
    message( STATUS "Virtualenv found at ${CATKIN_DEVEL_PREFIX}" )
  endif( NOT EXISTS "${activate_full}" )
