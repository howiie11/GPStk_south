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
include examples/CMakeFiles/example6.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example6.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example6.dir/flags.make

examples/CMakeFiles/example6.dir/example6.cpp.o: examples/CMakeFiles/example6.dir/flags.make
examples/CMakeFiles/example6.dir/example6.cpp.o: ../examples/example6.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shjzhang/Documents/Develop/rocket/pppbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/CMakeFiles/example6.dir/example6.cpp.o"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/example6.dir/example6.cpp.o -c /Users/shjzhang/Documents/Develop/rocket/pppbox/examples/example6.cpp

examples/CMakeFiles/example6.dir/example6.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example6.dir/example6.cpp.i"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shjzhang/Documents/Develop/rocket/pppbox/examples/example6.cpp > CMakeFiles/example6.dir/example6.cpp.i

examples/CMakeFiles/example6.dir/example6.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example6.dir/example6.cpp.s"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shjzhang/Documents/Develop/rocket/pppbox/examples/example6.cpp -o CMakeFiles/example6.dir/example6.cpp.s

examples/CMakeFiles/example6.dir/example6.cpp.o.requires:
.PHONY : examples/CMakeFiles/example6.dir/example6.cpp.o.requires

examples/CMakeFiles/example6.dir/example6.cpp.o.provides: examples/CMakeFiles/example6.dir/example6.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/example6.dir/build.make examples/CMakeFiles/example6.dir/example6.cpp.o.provides.build
.PHONY : examples/CMakeFiles/example6.dir/example6.cpp.o.provides

examples/CMakeFiles/example6.dir/example6.cpp.o.provides.build: examples/CMakeFiles/example6.dir/example6.cpp.o

# Object files for target example6
example6_OBJECTS = \
"CMakeFiles/example6.dir/example6.cpp.o"

# External object files for target example6
example6_EXTERNAL_OBJECTS =

examples/example6: examples/CMakeFiles/example6.dir/example6.cpp.o
examples/example6: examples/CMakeFiles/example6.dir/build.make
examples/example6: libpppbox.dylib
examples/example6: examples/CMakeFiles/example6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable example6"
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example6.dir/build: examples/example6
.PHONY : examples/CMakeFiles/example6.dir/build

examples/CMakeFiles/example6.dir/requires: examples/CMakeFiles/example6.dir/example6.cpp.o.requires
.PHONY : examples/CMakeFiles/example6.dir/requires

examples/CMakeFiles/example6.dir/clean:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example6.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example6.dir/clean

examples/CMakeFiles/example6.dir/depend:
	cd /Users/shjzhang/Documents/Develop/rocket/pppbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shjzhang/Documents/Develop/rocket/pppbox /Users/shjzhang/Documents/Develop/rocket/pppbox/examples /Users/shjzhang/Documents/Develop/rocket/pppbox/build /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples /Users/shjzhang/Documents/Develop/rocket/pppbox/build/examples/CMakeFiles/example6.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example6.dir/depend

