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
include apps/mergetools/CMakeFiles/mergeRinNav.dir/depend.make

# Include the progress variables for this target.
include apps/mergetools/CMakeFiles/mergeRinNav.dir/progress.make

# Include the compile flags for this target's objects.
include apps/mergetools/CMakeFiles/mergeRinNav.dir/flags.make

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o: apps/mergetools/CMakeFiles/mergeRinNav.dir/flags.make
apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o: ../apps/mergetools/mergeRinNav.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/mergetools/mergeRinNav.cpp

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/mergetools/mergeRinNav.cpp > CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.i

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/mergetools/mergeRinNav.cpp -o CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.s

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.requires:
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.requires

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.provides: apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.requires
	$(MAKE) -f apps/mergetools/CMakeFiles/mergeRinNav.dir/build.make apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.provides.build
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.provides

apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.provides.build: apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o

# Object files for target mergeRinNav
mergeRinNav_OBJECTS = \
"CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o"

# External object files for target mergeRinNav
mergeRinNav_EXTERNAL_OBJECTS =

apps/mergetools/mergeRinNav: apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o
apps/mergetools/mergeRinNav: apps/mergetools/CMakeFiles/mergeRinNav.dir/build.make
apps/mergetools/mergeRinNav: libpppbox.dylib
apps/mergetools/mergeRinNav: apps/mergetools/CMakeFiles/mergeRinNav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable mergeRinNav"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mergeRinNav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/mergetools/CMakeFiles/mergeRinNav.dir/build: apps/mergetools/mergeRinNav
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/build

apps/mergetools/CMakeFiles/mergeRinNav.dir/requires: apps/mergetools/CMakeFiles/mergeRinNav.dir/mergeRinNav.cpp.o.requires
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/requires

apps/mergetools/CMakeFiles/mergeRinNav.dir/clean:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools && $(CMAKE_COMMAND) -P CMakeFiles/mergeRinNav.dir/cmake_clean.cmake
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/clean

apps/mergetools/CMakeFiles/mergeRinNav.dir/depend:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shjzhang/Documents/Develop/rocket/pppbox /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/mergetools /Users/shjzhang/Documents/Develop/rocket/pppbox/build /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/mergetools/CMakeFiles/mergeRinNav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/mergetools/CMakeFiles/mergeRinNav.dir/depend

