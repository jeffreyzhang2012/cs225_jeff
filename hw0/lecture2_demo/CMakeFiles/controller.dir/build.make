# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/jezmond/snap/SAI/apps/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jezmond/snap/SAI/apps/cs225a/hw0

# Include any dependencies generated for this target.
include lecture2_demo/CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include lecture2_demo/CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include lecture2_demo/CMakeFiles/controller.dir/flags.make

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o: lecture2_demo/CMakeFiles/controller.dir/flags.make
lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o: ../lecture2_demo/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/hw0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o"
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/controller.cpp.o -c /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/controller.cpp

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/controller.cpp.i"
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/controller.cpp > CMakeFiles/controller.dir/controller.cpp.i

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/controller.cpp.s"
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/controller.cpp -o CMakeFiles/controller.dir/controller.cpp.s

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.requires:

.PHONY : lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.requires

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.provides: lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.requires
	$(MAKE) -f lecture2_demo/CMakeFiles/controller.dir/build.make lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.provides.build
.PHONY : lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.provides

lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.provides.build: lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o


# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/controller.cpp.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

../bin/lecture2_demo/controller: lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o
../bin/lecture2_demo/controller: lecture2_demo/CMakeFiles/controller.dir/build.make
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-common/build/libsai2-common.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-simulation/build/libsai2-simulation.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-model/build/libsai2-model.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-graphics/build/libsai2-graphics.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/lecture2_demo/controller: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2_demo/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/lecture2_demo/controller: lecture2_demo/CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/hw0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/lecture2_demo/controller"
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lecture2_demo/CMakeFiles/controller.dir/build: ../bin/lecture2_demo/controller

.PHONY : lecture2_demo/CMakeFiles/controller.dir/build

lecture2_demo/CMakeFiles/controller.dir/requires: lecture2_demo/CMakeFiles/controller.dir/controller.cpp.o.requires

.PHONY : lecture2_demo/CMakeFiles/controller.dir/requires

lecture2_demo/CMakeFiles/controller.dir/clean:
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo && $(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : lecture2_demo/CMakeFiles/controller.dir/clean

lecture2_demo/CMakeFiles/controller.dir/depend:
	cd /home/jezmond/snap/SAI/apps/cs225a/hw0 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jezmond/snap/SAI/apps/cs225a /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo /home/jezmond/snap/SAI/apps/cs225a/hw0 /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo /home/jezmond/snap/SAI/apps/cs225a/hw0/lecture2_demo/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lecture2_demo/CMakeFiles/controller.dir/depend

