# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(synthetic_task_generator_CONFIG_INCLUDED)
  return()
endif()
set(synthetic_task_generator_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(synthetic_task_generator_SOURCE_PREFIX /home/hypark/git/instance_contention_research/src/synthetic_task_generator)
  set(synthetic_task_generator_DEVEL_PREFIX /home/hypark/git/instance_contention_research/devel)
  set(synthetic_task_generator_INSTALL_PREFIX "")
  set(synthetic_task_generator_PREFIX ${synthetic_task_generator_DEVEL_PREFIX})
else()
  set(synthetic_task_generator_SOURCE_PREFIX "")
  set(synthetic_task_generator_DEVEL_PREFIX "")
  set(synthetic_task_generator_INSTALL_PREFIX /home/hypark/git/instance_contention_research/install)
  set(synthetic_task_generator_PREFIX ${synthetic_task_generator_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'synthetic_task_generator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(synthetic_task_generator_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/hypark/git/instance_contention_research/src/synthetic_task_generator/include " STREQUAL " ")
  set(synthetic_task_generator_INCLUDE_DIRS "")
  set(_include_dirs "/home/hypark/git/instance_contention_research/src/synthetic_task_generator/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'hypark <kite9240@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${synthetic_task_generator_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'synthetic_task_generator' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'synthetic_task_generator' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/hypark/git/instance_contention_research/src/synthetic_task_generator/${idir}'.  ${_report}")
    endif()
    _list_append_unique(synthetic_task_generator_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND synthetic_task_generator_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND synthetic_task_generator_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT synthetic_task_generator_NUM_DUMMY_TARGETS)
      set(synthetic_task_generator_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::synthetic_task_generator::wrapped-linker-option${synthetic_task_generator_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR synthetic_task_generator_NUM_DUMMY_TARGETS "${synthetic_task_generator_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::synthetic_task_generator::wrapped-linker-option${synthetic_task_generator_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND synthetic_task_generator_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND synthetic_task_generator_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND synthetic_task_generator_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/hypark/git/instance_contention_research/devel/lib;/home/hypark/git/instance_contention_research/devel/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/integrated_viewer/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ymc/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/xsens_driver/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vlg22c_cam/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vision_lane_detect/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vision_darknet_detect/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vision_beyond_track/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vel_pose_diff_checker/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vehicle_description/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_local_planner/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_global_planner/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_ros_helpers/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_simu/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_planner/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/op_utility/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/lidar_euclidean_cluster_detect/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vector_map_server/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/object_map/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/map_file/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/libvectormap/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/imm_ukf_pda_track/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vector_map/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vector_map_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/vectacam/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/twist_generator/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/twist_gate/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/twist_filter/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/tablet_socket/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/runtime_manager/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/tablet_socket_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/sick_lms5xx/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/sick_ldmrs_tools/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/sick_ldmrs_driver/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/sick_ldmrs_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/sick_ldmrs_description/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/range_vision_fusion/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/pure_pursuit/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/points_preprocessor/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/points_downsampler/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/lidar_localizer/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ndt_gpu/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_connector/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/rubis_lib/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/rubis_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_health_checker/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/as/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ros_observer/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/pcl_omp_registration/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/nmea_navsat/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ndt_tku/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ndt_cpu/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/microstrain_driver/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/memsic_imu/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/map_tools/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/map_tf_generator/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/log_tools/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/libwaypoint_follower/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/lgsvl_simulator_bridge/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/kvaser/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/javad_navsat_driver/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/image_processor/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/hokuyo/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/gnss/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/garmin/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ds4_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/ds4_driver/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/detected_objects_visualizer/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/custom_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/can_data_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/calibration_publisher/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_system_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_pointgrey_drivers/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_driveworks_interface/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_camera_lidar_calibrator/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/amathutils_lib/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_launcher_rviz/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_launcher/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_driveworks_gmsl_interface/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_config_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_can_msgs/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_build_flags/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/autoware_bag_tools/lib;/home/hypark/git/Autoware_On_Embedded/autoware.ai/install/adi_driver/lib;/home/hypark/git/Autoware_On_Embedded/rubis_ws/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(synthetic_task_generator_LIBRARY_DIRS ${lib_path})
      list(APPEND synthetic_task_generator_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'synthetic_task_generator'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND synthetic_task_generator_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(synthetic_task_generator_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${synthetic_task_generator_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 synthetic_task_generator_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${synthetic_task_generator_dep}_FOUND)
      find_package(${synthetic_task_generator_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${synthetic_task_generator_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(synthetic_task_generator_INCLUDE_DIRS ${${synthetic_task_generator_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(synthetic_task_generator_LIBRARIES ${synthetic_task_generator_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${synthetic_task_generator_dep}_LIBRARIES})
  _list_append_deduplicate(synthetic_task_generator_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(synthetic_task_generator_LIBRARIES ${synthetic_task_generator_LIBRARIES})

  _list_append_unique(synthetic_task_generator_LIBRARY_DIRS ${${synthetic_task_generator_dep}_LIBRARY_DIRS})
  list(APPEND synthetic_task_generator_EXPORTED_TARGETS ${${synthetic_task_generator_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${synthetic_task_generator_DIR}/${extra})
  endif()
  include(${extra})
endforeach()