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
CMAKE_BINARY_DIR = /home/jezmond/snap/SAI/apps/cs225a/bin

# Include any dependencies generated for this target.
include hw2/CMakeFiles/simviz_hw2.dir/depend.make

# Include the progress variables for this target.
include hw2/CMakeFiles/simviz_hw2.dir/progress.make

# Include the compile flags for this target's objects.
include hw2/CMakeFiles/simviz_hw2.dir/flags.make

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o: hw2/CMakeFiles/simviz_hw2.dir/flags.make
hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o: ../hw2/simviz_hw2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o -c /home/jezmond/snap/SAI/apps/cs225a/hw2/simviz_hw2.cpp

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.i"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jezmond/snap/SAI/apps/cs225a/hw2/simviz_hw2.cpp > CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.i

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.s"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jezmond/snap/SAI/apps/cs225a/hw2/simviz_hw2.cpp -o CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.s

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.requires:

.PHONY : hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.requires

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.provides: hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.requires
	$(MAKE) -f hw2/CMakeFiles/simviz_hw2.dir/build.make hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.provides.build
.PHONY : hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.provides

hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.provides.build: hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o


# Object files for target simviz_hw2
simviz_hw2_OBJECTS = \
"CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o"

# External object files for target simviz_hw2
simviz_hw2_EXTERNAL_OBJECTS =

hw2/simviz_hw2: hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o
hw2/simviz_hw2: hw2/CMakeFiles/simviz_hw2.dir/build.make
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-common/build/libsai2-common.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libGL.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libGLU.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-simulation/build/libsai2-simulation.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-model/build/libsai2-model.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-graphics/build/libsai2-graphics.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libGL.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libGLU.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libhiredis.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libglfw.so
hw2/simviz_hw2: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libhiredis.so
hw2/simviz_hw2: /usr/lib/x86_64-linux-gnu/libglfw.so
hw2/simviz_hw2: hw2/CMakeFiles/simviz_hw2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable simviz_hw2"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz_hw2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hw2/CMakeFiles/simviz_hw2.dir/build: hw2/simviz_hw2

.PHONY : hw2/CMakeFiles/simviz_hw2.dir/build

hw2/CMakeFiles/simviz_hw2.dir/requires: hw2/CMakeFiles/simviz_hw2.dir/simviz_hw2.cpp.o.requires

.PHONY : hw2/CMakeFiles/simviz_hw2.dir/requires

hw2/CMakeFiles/simviz_hw2.dir/clean:
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 && $(CMAKE_COMMAND) -P CMakeFiles/simviz_hw2.dir/cmake_clean.cmake
.PHONY : hw2/CMakeFiles/simviz_hw2.dir/clean

hw2/CMakeFiles/simviz_hw2.dir/depend:
	cd /home/jezmond/snap/SAI/apps/cs225a/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jezmond/snap/SAI/apps/cs225a /home/jezmond/snap/SAI/apps/cs225a/hw2 /home/jezmond/snap/SAI/apps/cs225a/bin /home/jezmond/snap/SAI/apps/cs225a/bin/hw2 /home/jezmond/snap/SAI/apps/cs225a/bin/hw2/CMakeFiles/simviz_hw2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hw2/CMakeFiles/simviz_hw2.dir/depend

