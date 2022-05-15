# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

yarp_configure_external_installation(SoftTerrainWalking)

macro(install_ini_files parent_dir)
  # Get list of models
  subdirlist(robots ${parent_dir}/robots/)

  # Install each model
  foreach (robot ${robots})
    file(GLOB scripts ${parent_dir}/robots/${robot}/*.ini)
    yarp_install(FILES ${scripts} DESTINATION ${SOFTTERRAINWALKING_ROBOTS_INSTALL_DIR}/${robot})

    subdirlist(subdirs ${parent_dir}/robots/${robot}/)
    foreach (subdir ${subdirs})
      yarp_install(DIRECTORY ${parent_dir}/robots/${robot}/${subdir} DESTINATION ${SOFTTERRAINWALKING_ROBOTS_INSTALL_DIR}/${robot})
    endforeach ()
  endforeach ()
endmacro()
