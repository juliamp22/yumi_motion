# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build

# Include any dependencies generated for this target.
include CMakeFiles/mt_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mt_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mt_demo.dir/flags.make

CMakeFiles/mt_demo.dir/mt_demo.o: CMakeFiles/mt_demo.dir/flags.make
CMakeFiles/mt_demo.dir/mt_demo.o: ../mt_demo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mt_demo.dir/mt_demo.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mt_demo.dir/mt_demo.o -c /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/mt_demo.cpp

CMakeFiles/mt_demo.dir/mt_demo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mt_demo.dir/mt_demo.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/mt_demo.cpp > CMakeFiles/mt_demo.dir/mt_demo.i

CMakeFiles/mt_demo.dir/mt_demo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mt_demo.dir/mt_demo.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/mt_demo.cpp -o CMakeFiles/mt_demo.dir/mt_demo.s

CMakeFiles/mt_demo.dir/mt_demo.o.requires:
.PHONY : CMakeFiles/mt_demo.dir/mt_demo.o.requires

CMakeFiles/mt_demo.dir/mt_demo.o.provides: CMakeFiles/mt_demo.dir/mt_demo.o.requires
	$(MAKE) -f CMakeFiles/mt_demo.dir/build.make CMakeFiles/mt_demo.dir/mt_demo.o.provides.build
.PHONY : CMakeFiles/mt_demo.dir/mt_demo.o.provides

CMakeFiles/mt_demo.dir/mt_demo.o.provides.build: CMakeFiles/mt_demo.dir/mt_demo.o

# Object files for target mt_demo
mt_demo_OBJECTS = \
"CMakeFiles/mt_demo.dir/mt_demo.o"

# External object files for target mt_demo
mt_demo_EXTERNAL_OBJECTS =

mt_demo: CMakeFiles/mt_demo.dir/mt_demo.o
mt_demo: CMakeFiles/mt_demo.dir/build.make
mt_demo: CMakeFiles/mt_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable mt_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mt_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mt_demo.dir/build: mt_demo
.PHONY : CMakeFiles/mt_demo.dir/build

CMakeFiles/mt_demo.dir/requires: CMakeFiles/mt_demo.dir/mt_demo.o.requires
.PHONY : CMakeFiles/mt_demo.dir/requires

CMakeFiles/mt_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mt_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mt_demo.dir/clean

CMakeFiles/mt_demo.dir/depend:
	cd /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build /home/users/julia.marsal/catkin_ws/src/yumi_motion/mt/build/CMakeFiles/mt_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mt_demo.dir/depend

