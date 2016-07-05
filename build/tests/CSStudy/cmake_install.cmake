# Install script for directory: /Users/zhaolei/pppbox/tests/CSStudy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/zhaolei/pppbox")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/Users/zhaolei/pppbox/build/tests/CSStudy/CSStudy")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CSStudy" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CSStudy")
    execute_process(COMMAND "/usr/bin/install_name_tool"
      -change "/Users/zhaolei/pppbox/build/libpppbox.dylib" "/Users/zhaolei/pppbox/lib/libpppbox.dylib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CSStudy")
    execute_process(COMMAND /usr/bin/install_name_tool
      -add_rpath "/Users/zhaolei/pppbox:$ORIGIN/../lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CSStudy")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CSStudy")
    endif()
  endif()
endif()

