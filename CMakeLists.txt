cmake_minimum_required(VERSION 3.16)

  project(V4L2_TEST LANGUAGES CXX)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  find_package(PkgConfig REQUIRED)
  find_path(
      ROCKCHIP_MPP_INCLUDE_DIR
      NAMES rk_mpi.h
      PATH_SUFFIXES rockchip
  )

  find_library(
      ROCKCHIP_MPP_LIBRARY
      NAMES rockchip_mpp
  )


  pkg_check_modules(
      GSTREAMER
      REQUIRED
      IMPORTED_TARGET
      gstreamer-1.0
      gstreamer-app-1.0
  )

  add_executable(
      v4l2_video
      src/v4l2_video.cpp
  )

    add_executable(
      v4l2_capture
      src/v4l2_capture.cpp
  )

   if(NOT ROCKCHIP_MPP_INCLUDE_DIR OR NOT ROCKCHIP_MPP_LIBRARY)
      message(FATAL_ERROR "Rockchip MPP development files not found")
  endif()

   target_include_directories(
      v4l2_video PRIVATE
      ${ROCKCHIP_MPP_INCLUDE_DIR}
  )



  target_compile_options(
      v4l2_video
      PRIVATE
      -Wall
      -Wextra
      -O2
      -fpermissive

  )

  target_link_libraries(
      v4l2_video
      PRIVATE
      PkgConfig::GSTREAMER
      ${ROCKCHIP_MPP_LIBRARY}

  )
