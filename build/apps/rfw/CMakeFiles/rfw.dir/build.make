# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /sw/bin/cmake

# The command to remove a file.
RM = /sw/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /sw/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/shjzhang/Documents/Develop/rocket/pppbox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/shjzhang/Documents/Develop/rocket/pppbox/build

# Include any dependencies generated for this target.
include apps/rfw/CMakeFiles/rfw.dir/depend.make

# Include the progress variables for this target.
include apps/rfw/CMakeFiles/rfw.dir/progress.make

# Include the compile flags for this target's objects.
include apps/rfw/CMakeFiles/rfw.dir/flags.make

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o: apps/rfw/CMakeFiles/rfw.dir/flags.make
apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o: ../apps/rfw/rfw.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rfw.dir/rfw.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/rfw.cpp

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rfw.dir/rfw.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/rfw.cpp > CMakeFiles/rfw.dir/rfw.cpp.i

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rfw.dir/rfw.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/rfw.cpp -o CMakeFiles/rfw.dir/rfw.cpp.s

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.requires:
.PHONY : apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.requires

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.provides: apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.requires
	$(MAKE) -f apps/rfw/CMakeFiles/rfw.dir/build.make apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.provides.build
.PHONY : apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.provides

apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.provides.build: apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o: apps/rfw/CMakeFiles/rfw.dir/flags.make
apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o: ../apps/rfw/FDStreamBuff.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rfw.dir/FDStreamBuff.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/FDStreamBuff.cpp

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rfw.dir/FDStreamBuff.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/FDStreamBuff.cpp > CMakeFiles/rfw.dir/FDStreamBuff.cpp.i

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rfw.dir/FDStreamBuff.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/FDStreamBuff.cpp -o CMakeFiles/rfw.dir/FDStreamBuff.cpp.s

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.requires:
.PHONY : apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.requires

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.provides: apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.requires
	$(MAKE) -f apps/rfw/CMakeFiles/rfw.dir/build.make apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.provides.build
.PHONY : apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.provides

apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.provides.build: apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o: apps/rfw/CMakeFiles/rfw.dir/flags.make
apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o: ../apps/rfw/TCPStreamBuff.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/TCPStreamBuff.cpp

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rfw.dir/TCPStreamBuff.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/TCPStreamBuff.cpp > CMakeFiles/rfw.dir/TCPStreamBuff.cpp.i

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rfw.dir/TCPStreamBuff.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw/TCPStreamBuff.cpp -o CMakeFiles/rfw.dir/TCPStreamBuff.cpp.s

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.requires:
.PHONY : apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.requires

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.provides: apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.requires
	$(MAKE) -f apps/rfw/CMakeFiles/rfw.dir/build.make apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.provides.build
.PHONY : apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.provides

apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.provides.build: apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o

# Object files for target rfw
rfw_OBJECTS = \
"CMakeFiles/rfw.dir/rfw.cpp.o" \
"CMakeFiles/rfw.dir/FDStreamBuff.cpp.o" \
"CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o"

# External object files for target rfw
rfw_EXTERNAL_OBJECTS =

apps/rfw/rfw: apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o
apps/rfw/rfw: apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o
apps/rfw/rfw: apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o
apps/rfw/rfw: apps/rfw/CMakeFiles/rfw.dir/build.make
apps/rfw/rfw: libpppbox.dylib
apps/rfw/rfw: apps/rfw/CMakeFiles/rfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable rfw"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rfw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/rfw/CMakeFiles/rfw.dir/build: apps/rfw/rfw
.PHONY : apps/rfw/CMakeFiles/rfw.dir/build

apps/rfw/CMakeFiles/rfw.dir/requires: apps/rfw/CMakeFiles/rfw.dir/rfw.cpp.o.requires
apps/rfw/CMakeFiles/rfw.dir/requires: apps/rfw/CMakeFiles/rfw.dir/FDStreamBuff.cpp.o.requires
apps/rfw/CMakeFiles/rfw.dir/requires: apps/rfw/CMakeFiles/rfw.dir/TCPStreamBuff.cpp.o.requires
.PHONY : apps/rfw/CMakeFiles/rfw.dir/requires

apps/rfw/CMakeFiles/rfw.dir/clean:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw && $(CMAKE_COMMAND) -P CMakeFiles/rfw.dir/cmake_clean.cmake
.PHONY : apps/rfw/CMakeFiles/rfw.dir/clean

apps/rfw/CMakeFiles/rfw.dir/depend:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shjzhang/Documents/Develop/rocket/pppbox /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/rfw /Users/shjzhang/Documents/Develop/rocket/pppbox/build /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/rfw/CMakeFiles/rfw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/rfw/CMakeFiles/rfw.dir/depend

