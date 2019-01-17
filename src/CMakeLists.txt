# All cpp files for UTCORE
FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

# https://github.com/joebedard/ORB_SLAM2_TEAM
find_package(OrbSlam2Team REQUIRED)

# https://github.com/stevenlovegrove/Pangolin
find_package(Pangolin REQUIRED)

# ZeroMQ cpp bindings
# https://github.com/zeromq/cppzmq
find_package(cppzmq 3.4 REQUIRED)

get_target_property(coreIncDirs CONAN_PKG::ubitrack_component_core INTERFACE_INCLUDE_DIRECTORIES)

foreach(fpath ${UBITRACK_SRCS})
   GET_FILENAME_COMPONENT(fname ${fpath} NAME_WE)
   set(UBITRACK_LIBRARY_NAME ${fname})

   add_library(${UBITRACK_LIBRARY_NAME} SHARED ${fpath})

   target_link_libraries(${UBITRACK_LIBRARY_NAME} 
      CONAN_PKG::ubitrack_core
      CONAN_PKG::ubitrack_vision
      CONAN_PKG::ubitrack_dataflow
      ${OrbSlam2Team_LIBRARIES}
      ${Pangolin_LIBRARIES}
      cppzmq-static
   )

   target_include_directories(${UBITRACK_LIBRARY_NAME} PRIVATE
      ${coreIncDirs}
      ${OrbSlam2Team_INCLUDE_DIRS}
      ${Pangolin_INCLUDE_DIR}
      ${cppzmq_INCLUDE_DIR}
   )

   # do not create symlinks to versioned names of component libraries
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

   # Apply Target Properties
   if(MSVC)
      if(CMAKE_CROSSCOMPILING)
         set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
      endif()
      set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
   endif()

   ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
   ubitrack_install_component(${UBITRACK_LIBRARY_NAME})

endforeach()

FILE(GLOB UBITRACK_HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)
foreach(hdr ${UBITRACK_HEADERS})
  string(REGEX REPLACE "${CMAKE_BINARY_DIR}/" "" hdr2 "${hdr}")
  GET_FILENAME_COMPONENT(fpath ${hdr2} PATH)
  IF(fpath)
    install(FILES ${hdr} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/utComponents/${fpath}" COMPONENT dev)
  ELSE(fpath)
    install(FILES ${hdr} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/utComponents" COMPONENT dev)
  ENDIF(fpath)
endforeach()
