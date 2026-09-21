cmake_minimum_required(VERSION 3.16)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" ROBOBATON_RELEASE_VERSION)
string(STRIP "${ROBOBATON_RELEASE_VERSION}" ROBOBATON_RELEASE_VERSION)
if(NOT ROBOBATON_RELEASE_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
  message(FATAL_ERROR "VERSION must contain a SemVer core version")
endif()
project(robobaton_4p_ros2_demo VERSION ${ROBOBATON_RELEASE_VERSION} LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Release>:-DNDEBUG>
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep target ROS discovery build-local so the merged install remains relocatable.
set(X5_BUILD_AMENT_PREFIX_PATH "" CACHE STRING
  "Build-only AMENT_PREFIX_PATH used to discover target ROS resources")
if(X5_BUILD_AMENT_PREFIX_PATH)
  set(ENV{AMENT_PREFIX_PATH} "${X5_BUILD_AMENT_PREFIX_PATH}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Threads REQUIRED)

set(X5_MEDIA_CODEC_INCLUDE_DIR "${CMAKE_SYSROOT}/usr_x5/include" CACHE PATH
  "X5 media-codec include directory")
set(X5_MULTIMEDIA_LIBRARY "${CMAKE_SYSROOT}/usr_x5/hobot/lib/libmultimedia.so.1" CACHE FILEPATH
  "X5 multimedia runtime library")
set(X5_HBMEM_LIBRARY "${CMAKE_SYSROOT}/usr_x5/hobot/lib/libhbmem.so.1" CACHE FILEPATH
  "X5 hbmem runtime library")
set(X5_ALOG_LIBRARY "${CMAKE_SYSROOT}/usr_x5/hobot/lib/libalog.so.1" CACHE FILEPATH
  "X5 alog runtime library")
foreach(required_x5_media_path
    "${X5_MEDIA_CODEC_INCLUDE_DIR}/hb_media_codec.h"
    "${X5_MULTIMEDIA_LIBRARY}" "${X5_HBMEM_LIBRARY}" "${X5_ALOG_LIBRARY}")
  if(NOT EXISTS "${required_x5_media_path}")
    message(FATAL_ERROR "Missing X5 media-codec dependency: ${required_x5_media_path}")
  endif()
endforeach()
add_library(X5MediaCodec::Multimedia SHARED IMPORTED GLOBAL)
set_target_properties(X5MediaCodec::Multimedia PROPERTIES
  IMPORTED_LOCATION "${X5_MULTIMEDIA_LIBRARY}"
  IMPORTED_SONAME "libmultimedia.so.1"
  INTERFACE_INCLUDE_DIRECTORIES "${X5_MEDIA_CODEC_INCLUDE_DIR}")
add_library(X5MediaCodec::Hbmem SHARED IMPORTED GLOBAL)
set_target_properties(X5MediaCodec::Hbmem PROPERTIES
  IMPORTED_LOCATION "${X5_HBMEM_LIBRARY}"
  IMPORTED_SONAME "libhbmem.so.1")
add_library(X5MediaCodec::Alog SHARED IMPORTED GLOBAL)
set_target_properties(X5MediaCodec::Alog PROPERTIES
  IMPORTED_LOCATION "${X5_ALOG_LIBRARY}"
  IMPORTED_SONAME "libalog.so.1")

get_filename_component(DEFAULT_ROBOBATON_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib" ABSOLUTE)
set(ROBOBATON_LIB_DIR "${DEFAULT_ROBOBATON_LIB_DIR}" CACHE PATH
  "Existing package library directory containing synchronized ABI-v2 producers")

set(ICM42688_REAL "${ROBOBATON_LIB_DIR}/libicm42688.so.2.1.0")
set(ICM42688_SONAME "${ROBOBATON_LIB_DIR}/libicm42688.so.2")
set(ICM42688_LINKER "${ROBOBATON_LIB_DIR}/libicm42688.so")
set(SC132_REAL "${ROBOBATON_LIB_DIR}/libsc132.so.2.0.1")
set(SC132_SONAME "${ROBOBATON_LIB_DIR}/libsc132.so.2")
set(SC132_LINKER "${ROBOBATON_LIB_DIR}/libsc132.so")

foreach(required_path
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/icm42688_driver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/sc132camera.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_common.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_config.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_common.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_config.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/timestamp_mapper.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/timestamp_mapper.cpp"
    "${ICM42688_REAL}" "${ICM42688_SONAME}" "${ICM42688_LINKER}"
    "${SC132_REAL}" "${SC132_SONAME}" "${SC132_LINKER}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Missing required RoboBaton ABI-v2 artifact: ${required_path}")
  endif()
endforeach()

add_library(robobaton_icm42688 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_icm42688 PROPERTIES
  IMPORTED_LOCATION "${ICM42688_REAL}"
  IMPORTED_SONAME "libicm42688.so.2"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(robobaton_sc132 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_sc132 PROPERTIES
  IMPORTED_LOCATION "${SC132_REAL}"
  IMPORTED_SONAME "libsc132.so.2"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(robobaton_nv12_compressed_image_transport SHARED
  src/nv12_compressed_publisher.cpp)
ament_target_dependencies(robobaton_nv12_compressed_image_transport
  rclcpp rcl_interfaces sensor_msgs image_transport pluginlib)
target_link_libraries(robobaton_nv12_compressed_image_transport
  "-Wl,--no-as-needed"
  X5MediaCodec::Multimedia
  X5MediaCodec::Hbmem
  X5MediaCodec::Alog
  "-Wl,--as-needed")
target_compile_options(robobaton_nv12_compressed_image_transport PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(robobaton_nv12_compressed_image_transport PRIVATE
  ROBOBATON_RELEASE_VERSION="${ROBOBATON_RELEASE_VERSION}")
target_link_options(robobaton_nv12_compressed_image_transport PRIVATE
  "LINKER:-z,defs" "LINKER:--as-needed")
set_target_properties(robobaton_nv12_compressed_image_transport PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "\$ORIGIN;/usr/hobot/lib;/usr/lib;/lib64;/lib")
pluginlib_export_plugin_description_file(image_transport robobaton_nv12_compressed_plugins.xml)

add_executable(robobaton_sensors_node
  src/camera_publisher.cpp
  src/imu_publisher.cpp
  src/publication_rate_metrics.cpp
  src/timestamp_mapper.cpp
  src/robobaton_sensors_node.cpp
  src/cam_demo_common.cpp
  src/cam_demo_config.cpp)

ament_target_dependencies(robobaton_sensors_node rclcpp sensor_msgs image_transport)
target_include_directories(robobaton_sensors_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(robobaton_sensors_node
  robobaton_icm42688
  robobaton_sc132
  Threads::Threads)
target_compile_options(robobaton_sensors_node PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(robobaton_sensors_node PRIVATE
  ROBOBATON_RELEASE_VERSION="${ROBOBATON_RELEASE_VERSION}")
target_link_options(robobaton_sensors_node PRIVATE
  "LINKER:-z,defs" "LINKER:--as-needed")

add_executable(robobaton_imu_rate_monitor
  src/imu_rate_monitor.cpp)
ament_target_dependencies(robobaton_imu_rate_monitor rclcpp sensor_msgs)
target_compile_options(robobaton_imu_rate_monitor PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(robobaton_imu_rate_monitor PRIVATE
  ROBOBATON_RELEASE_VERSION="${ROBOBATON_RELEASE_VERSION}")
target_link_options(robobaton_imu_rate_monitor PRIVATE
  "LINKER:-z,defs" "LINKER:--as-needed")

# Project producers resolve package-locally; canonical vendor search locations remain unchanged.
set_target_properties(robobaton_sensors_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "\$ORIGIN;/usr/hobot/lib;/usr/hobot/lib/sensor;/usr/lib;/lib64;/lib")
set_target_properties(robobaton_imu_rate_monitor PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "\$ORIGIN;/usr/lib;/lib64;/lib")

install(TARGETS robobaton_sensors_node robobaton_imu_rate_monitor RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS robobaton_nv12_compressed_image_transport LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES
  "${ICM42688_REAL}" "${ICM42688_SONAME}" "${ICM42688_LINKER}"
  "${SC132_REAL}" "${SC132_SONAME}" "${SC132_LINKER}"
  DESTINATION lib/${PROJECT_NAME})

set(X5_RUNTIME_LIB_DIR "" CACHE PATH
  "X5 runtime library directory providing canonical ROS2 transitive runtime libs")
if(NOT X5_RUNTIME_LIB_DIR AND CMAKE_SYSROOT AND
   EXISTS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libspdlog.so.1")
  set(X5_RUNTIME_LIB_DIR "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
endif()
set(X5_RUNTIME_LIBS
  libconsole_bridge.so libconsole_bridge.so.1.0
  libspdlog.so libspdlog.so.1 libspdlog.so.1.9.2
  libfmt.so libfmt.so.8 libfmt.so.8.1.1
  libtinyxml2.so libtinyxml2.so.9 libtinyxml2.so.9.0.0
  libssl.so libssl.so.3
  libcrypto.so libcrypto.so.3)
foreach(runtime_lib IN LISTS X5_RUNTIME_LIBS)
  if(NOT X5_RUNTIME_LIB_DIR OR NOT EXISTS "${X5_RUNTIME_LIB_DIR}/${runtime_lib}")
    message(FATAL_ERROR "Missing X5 runtime lib: ${runtime_lib}; pass -DX5_RUNTIME_LIB_DIR=<path>")
  endif()
  install(FILES "${X5_RUNTIME_LIB_DIR}/${runtime_lib}" DESTINATION lib/${PROJECT_NAME})
endforeach()
set(X5_NUMPY_RUNTIME_LIB_DIR "" CACHE PATH
  "X5 sysroot library directory providing NumPy runtime libraries for ROS2 Python CLI")
if(NOT X5_NUMPY_RUNTIME_LIB_DIR AND CMAKE_SYSROOT AND
   EXISTS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libblas.so.3")
  set(X5_NUMPY_RUNTIME_LIB_DIR "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
endif()
foreach(numpy_runtime_lib IN ITEMS libblas.so.3 liblapack.so.3 libgfortran.so.5)
  if(NOT X5_NUMPY_RUNTIME_LIB_DIR OR NOT EXISTS "${X5_NUMPY_RUNTIME_LIB_DIR}/${numpy_runtime_lib}")
    message(FATAL_ERROR "Missing X5 NumPy runtime lib: ${numpy_runtime_lib}; pass -DX5_NUMPY_RUNTIME_LIB_DIR=<path>")
  endif()
  get_filename_component(numpy_runtime_real "${X5_NUMPY_RUNTIME_LIB_DIR}/${numpy_runtime_lib}" REALPATH)
  if(NOT EXISTS "${numpy_runtime_real}")
    message(FATAL_ERROR "Missing resolved X5 NumPy runtime lib: ${numpy_runtime_real}")
  endif()
  install(FILES "${numpy_runtime_real}" DESTINATION lib/${PROJECT_NAME} RENAME ${numpy_runtime_lib})
endforeach()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_fastdds_profile.sh.in")
install(PROGRAMS script/robobaton_ros2_env.bash DESTINATION .)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES VERSION DESTINATION share/${PROJECT_NAME})

ament_package()
