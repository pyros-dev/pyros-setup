# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

macro(catkin_python_venv)
	set(PYTHON_VERSION "" CACHE STRING "Specify specific Python version to use ('major.minor' or 'major')")
	if(PYTHON_VERSION)
	  set(PythonInterp_FIND_VERSION "${PYTHON_VERSION}")
	endif()

	find_package(PythonInterp REQUIRED)

	# This sets the virtual environment directory to be the catkin devel space
	set( PythonVirtualenvHome ${CATKIN_DEVEL_PREFIX} )

	get_filename_component( activate_full "${PythonVirtualenvHome}/bin/activate"
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
		  "${PythonVirtualenvHome}"
		RESULT_VARIABLE failed
		ERROR_VARIABLE error )
	  if( failed )
		message( SEND_ERROR ${error} )
	  endif()
	else( NOT EXISTS "${activate_full}" )
	  message( STATUS "Virtualenv found at ${PythonVirtualenvHome}" )
	endif( NOT EXISTS "${activate_full}" )

	# TODO more : Testing ? IPython Notebooks ? here or Pyros ?
	#Inspiration from https://github.com/KitwareMedical/TubeTK/blob/master/CMake/TubeTKVirtualEnvSetup.cmake

	# Install requirements into the virtual environment
	execute_process( COMMAND "${PythonVirtualenvHome}/bin/pip"
		    install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
		  RESULT_VARIABLE failed
		  OUTPUT_VARIABLE output
		  ERROR_VARIABLE error )
		if( failed )
		  message( SEND_ERROR ${error} )
		else (failed)
		  message( STATUS ${output} )
		endif( failed )

	# Activating virtualenv
	message(STATUS "To use the virtual environment setup, use 'source ${PythonVirtualenvHome}/bin/activate'")
	# Automatically use the virtualenv via settign new python interpreter ?


endmacro(catkin_python_venv)

