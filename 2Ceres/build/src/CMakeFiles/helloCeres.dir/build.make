# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/noetic/module/2Ceres

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/noetic/module/2Ceres/build

# Include any dependencies generated for this target.
include src/CMakeFiles/helloCeres.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/helloCeres.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/helloCeres.dir/flags.make

src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.o: src/CMakeFiles/helloCeres.dir/flags.make
src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.o: ../src/1HelloCeres.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/noetic/module/2Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.o"
	cd /home/noetic/module/2Ceres/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helloCeres.dir/1HelloCeres.cc.o -c /home/noetic/module/2Ceres/src/1HelloCeres.cc

src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helloCeres.dir/1HelloCeres.cc.i"
	cd /home/noetic/module/2Ceres/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/noetic/module/2Ceres/src/1HelloCeres.cc > CMakeFiles/helloCeres.dir/1HelloCeres.cc.i

src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helloCeres.dir/1HelloCeres.cc.s"
	cd /home/noetic/module/2Ceres/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/noetic/module/2Ceres/src/1HelloCeres.cc -o CMakeFiles/helloCeres.dir/1HelloCeres.cc.s

# Object files for target helloCeres
helloCeres_OBJECTS = \
"CMakeFiles/helloCeres.dir/1HelloCeres.cc.o"

# External object files for target helloCeres
helloCeres_EXTERNAL_OBJECTS =

src/helloCeres: src/CMakeFiles/helloCeres.dir/1HelloCeres.cc.o
src/helloCeres: src/CMakeFiles/helloCeres.dir/build.make
src/helloCeres: /usr/local/lib/libceres.a
src/helloCeres: /usr/lib/x86_64-linux-gnu/libglog.so
src/helloCeres: /usr/lib/x86_64-linux-gnu/liblapack.so
src/helloCeres: /usr/lib/x86_64-linux-gnu/libblas.so
src/helloCeres: src/CMakeFiles/helloCeres.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/noetic/module/2Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable helloCeres"
	cd /home/noetic/module/2Ceres/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helloCeres.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/helloCeres.dir/build: src/helloCeres

.PHONY : src/CMakeFiles/helloCeres.dir/build

src/CMakeFiles/helloCeres.dir/clean:
	cd /home/noetic/module/2Ceres/build/src && $(CMAKE_COMMAND) -P CMakeFiles/helloCeres.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/helloCeres.dir/clean

src/CMakeFiles/helloCeres.dir/depend:
	cd /home/noetic/module/2Ceres/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/noetic/module/2Ceres /home/noetic/module/2Ceres/src /home/noetic/module/2Ceres/build /home/noetic/module/2Ceres/build/src /home/noetic/module/2Ceres/build/src/CMakeFiles/helloCeres.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/helloCeres.dir/depend

