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
include lecture2_demo/CMakeFiles/simviz.dir/depend.make

# Include the progress variables for this target.
include lecture2_demo/CMakeFiles/simviz.dir/progress.make

# Include the compile flags for this target's objects.
include lecture2_demo/CMakeFiles/simviz.dir/flags.make

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o: lecture2_demo/CMakeFiles/simviz.dir/flags.make
lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o: ../lecture2_demo/simviz.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simviz.dir/simviz.cpp.o -c /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/simviz.cpp

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz.dir/simviz.cpp.i"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/simviz.cpp > CMakeFiles/simviz.dir/simviz.cpp.i

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz.dir/simviz.cpp.s"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo/simviz.cpp -o CMakeFiles/simviz.dir/simviz.cpp.s

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.requires:

.PHONY : lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.requires

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.provides: lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.requires
	$(MAKE) -f lecture2_demo/CMakeFiles/simviz.dir/build.make lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.provides.build
.PHONY : lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.provides

lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.provides.build: lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o


# Object files for target simviz
simviz_OBJECTS = \
"CMakeFiles/simviz.dir/simviz.cpp.o"

# External object files for target simviz
simviz_EXTERNAL_OBJECTS =

lecture2_demo/simviz: lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o
lecture2_demo/simviz: lecture2_demo/CMakeFiles/simviz.dir/build.make
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-common/build/libsai2-common.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libGL.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libGLU.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-simulation/build/libsai2-simulation.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-model/build/libsai2-model.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-graphics/build/libsai2-graphics.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/build/libsai2-urdf.a
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/chai3d/build/libchai3d.a
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libGL.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libGLU.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libglfw.so
lecture2_demo/simviz: /home/jezmond/snap/SAI/core/sai2-model/rbdl/build/librbdl.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libhiredis.so
lecture2_demo/simviz: /usr/lib/x86_64-linux-gnu/libglfw.so
lecture2_demo/simviz: lecture2_demo/CMakeFiles/simviz.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jezmond/snap/SAI/apps/cs225a/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable simviz"
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lecture2_demo/CMakeFiles/simviz.dir/build: lecture2_demo/simviz

.PHONY : lecture2_demo/CMakeFiles/simviz.dir/build

lecture2_demo/CMakeFiles/simviz.dir/requires: lecture2_demo/CMakeFiles/simviz.dir/simviz.cpp.o.requires

.PHONY : lecture2_demo/CMakeFiles/simviz.dir/requires

lecture2_demo/CMakeFiles/simviz.dir/clean:
	cd /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo && $(CMAKE_COMMAND) -P CMakeFiles/simviz.dir/cmake_clean.cmake
.PHONY : lecture2_demo/CMakeFiles/simviz.dir/clean

lecture2_demo/CMakeFiles/simviz.dir/depend:
	cd /home/jezmond/snap/SAI/apps/cs225a/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jezmond/snap/SAI/apps/cs225a /home/jezmond/snap/SAI/apps/cs225a/lecture2_demo /home/jezmond/snap/SAI/apps/cs225a/bin /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo /home/jezmond/snap/SAI/apps/cs225a/bin/lecture2_demo/CMakeFiles/simviz.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lecture2_demo/CMakeFiles/simviz.dir/depend

