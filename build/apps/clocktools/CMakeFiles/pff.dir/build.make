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
include apps/clocktools/CMakeFiles/pff.dir/depend.make

# Include the progress variables for this target.
include apps/clocktools/CMakeFiles/pff.dir/progress.make

# Include the compile flags for this target's objects.
include apps/clocktools/CMakeFiles/pff.dir/flags.make

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o: apps/clocktools/CMakeFiles/pff.dir/flags.make
apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o: ../apps/clocktools/pff.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pff.dir/pff.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/clocktools/pff.cpp

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pff.dir/pff.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/clocktools/pff.cpp > CMakeFiles/pff.dir/pff.cpp.i

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pff.dir/pff.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/clocktools/pff.cpp -o CMakeFiles/pff.dir/pff.cpp.s

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.requires:
.PHONY : apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.requires

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.provides: apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.requires
	$(MAKE) -f apps/clocktools/CMakeFiles/pff.dir/build.make apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.provides.build
.PHONY : apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.provides

apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.provides.build: apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o

# Object files for target pff
pff_OBJECTS = \
"CMakeFiles/pff.dir/pff.cpp.o"

# External object files for target pff
pff_EXTERNAL_OBJECTS =

apps/clocktools/pff: apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o
apps/clocktools/pff: apps/clocktools/CMakeFiles/pff.dir/build.make
apps/clocktools/pff: libpppbox.dylib
apps/clocktools/pff: apps/clocktools/CMakeFiles/pff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pff"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/clocktools/CMakeFiles/pff.dir/build: apps/clocktools/pff
.PHONY : apps/clocktools/CMakeFiles/pff.dir/build

apps/clocktools/CMakeFiles/pff.dir/requires: apps/clocktools/CMakeFiles/pff.dir/pff.cpp.o.requires
.PHONY : apps/clocktools/CMakeFiles/pff.dir/requires

apps/clocktools/CMakeFiles/pff.dir/clean:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools && $(CMAKE_COMMAND) -P CMakeFiles/pff.dir/cmake_clean.cmake
.PHONY : apps/clocktools/CMakeFiles/pff.dir/clean

apps/clocktools/CMakeFiles/pff.dir/depend:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shjzhang/Documents/Develop/rocket/pppbox /Users/shjzhang/Documents/Develop/rocket/pppbox/apps/clocktools /Users/shjzhang/Documents/Develop/rocket/pppbox/build /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools /Users/shjzhang/Documents/Develop/rocket/pppbox/build/apps/clocktools/CMakeFiles/pff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/clocktools/CMakeFiles/pff.dir/depend

