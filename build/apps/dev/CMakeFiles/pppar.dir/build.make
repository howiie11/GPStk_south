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
include apps/dev/CMakeFiles/pppar.dir/depend.make

# Include the progress variables for this target.
include apps/dev/CMakeFiles/pppar.dir/progress.make

# Include the compile flags for this target's objects.
include apps/dev/CMakeFiles/pppar.dir/flags.make

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o: apps/dev/CMakeFiles/pppar.dir/flags.make
apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o: ../apps/dev/pppar.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pppar.dir/pppar.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/dev/pppar.cpp

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pppar.dir/pppar.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/dev/pppar.cpp > CMakeFiles/pppar.dir/pppar.cpp.i

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pppar.dir/pppar.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/dev/pppar.cpp -o CMakeFiles/pppar.dir/pppar.cpp.s

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.requires:
.PHONY : apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.requires

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.provides: apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.requires
	$(MAKE) -f apps/dev/CMakeFiles/pppar.dir/build.make apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.provides.build
.PHONY : apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.provides

apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.provides.build: apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o

# Object files for target pppar
pppar_OBJECTS = \
"CMakeFiles/pppar.dir/pppar.cpp.o"

# External object files for target pppar
pppar_EXTERNAL_OBJECTS =

apps/dev/pppar: apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o
apps/dev/pppar: apps/dev/CMakeFiles/pppar.dir/build.make
apps/dev/pppar: libpppbox.dylib
apps/dev/pppar: apps/dev/CMakeFiles/pppar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pppar"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pppar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/dev/CMakeFiles/pppar.dir/build: apps/dev/pppar
.PHONY : apps/dev/CMakeFiles/pppar.dir/build

apps/dev/CMakeFiles/pppar.dir/requires: apps/dev/CMakeFiles/pppar.dir/pppar.cpp.o.requires
.PHONY : apps/dev/CMakeFiles/pppar.dir/requires

apps/dev/CMakeFiles/pppar.dir/clean:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev && $(CMAKE_COMMAND) -P CMakeFiles/pppar.dir/cmake_clean.cmake
.PHONY : apps/dev/CMakeFiles/pppar.dir/clean

apps/dev/CMakeFiles/pppar.dir/depend:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shjzhang/Documents/Develop/rocket/pppbox /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/dev /Users/shjzhang/Documents/Develop/rocket/pppbox/build /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/dev/CMakeFiles/pppar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/dev/CMakeFiles/pppar.dir/depend

