# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.9

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2017.3.3\bin\cmake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2017.3.3\bin\cmake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\jaysh\CLionProjects\TelenavSensorFusion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/TelenavSensorFusion.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TelenavSensorFusion.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TelenavSensorFusion.dir/flags.make

CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj: CMakeFiles/TelenavSensorFusion.dir/flags.make
CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj"
	C:\msys64\mingw32\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\TelenavSensorFusion.dir\main.cpp.obj -c C:\Users\jaysh\CLionProjects\TelenavSensorFusion\main.cpp

CMakeFiles/TelenavSensorFusion.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelenavSensorFusion.dir/main.cpp.i"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\jaysh\CLionProjects\TelenavSensorFusion\main.cpp > CMakeFiles\TelenavSensorFusion.dir\main.cpp.i

CMakeFiles/TelenavSensorFusion.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelenavSensorFusion.dir/main.cpp.s"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\jaysh\CLionProjects\TelenavSensorFusion\main.cpp -o CMakeFiles\TelenavSensorFusion.dir\main.cpp.s

CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.requires:

.PHONY : CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.requires

CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.provides: CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.requires
	$(MAKE) -f CMakeFiles\TelenavSensorFusion.dir\build.make CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.provides.build
.PHONY : CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.provides

CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.provides.build: CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj


CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj: CMakeFiles/TelenavSensorFusion.dir/flags.make
CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj: ../tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj"
	C:\msys64\mingw32\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\TelenavSensorFusion.dir\tools.cpp.obj -c C:\Users\jaysh\CLionProjects\TelenavSensorFusion\tools.cpp

CMakeFiles/TelenavSensorFusion.dir/tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelenavSensorFusion.dir/tools.cpp.i"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\jaysh\CLionProjects\TelenavSensorFusion\tools.cpp > CMakeFiles\TelenavSensorFusion.dir\tools.cpp.i

CMakeFiles/TelenavSensorFusion.dir/tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelenavSensorFusion.dir/tools.cpp.s"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\jaysh\CLionProjects\TelenavSensorFusion\tools.cpp -o CMakeFiles\TelenavSensorFusion.dir\tools.cpp.s

CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.requires:

.PHONY : CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.requires

CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.provides: CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.requires
	$(MAKE) -f CMakeFiles\TelenavSensorFusion.dir\build.make CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.provides.build
.PHONY : CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.provides

CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.provides.build: CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj


CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj: CMakeFiles/TelenavSensorFusion.dir/flags.make
CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj: ../Iteration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj"
	C:\msys64\mingw32\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\TelenavSensorFusion.dir\Iteration.cpp.obj -c C:\Users\jaysh\CLionProjects\TelenavSensorFusion\Iteration.cpp

CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.i"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\jaysh\CLionProjects\TelenavSensorFusion\Iteration.cpp > CMakeFiles\TelenavSensorFusion.dir\Iteration.cpp.i

CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.s"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\jaysh\CLionProjects\TelenavSensorFusion\Iteration.cpp -o CMakeFiles\TelenavSensorFusion.dir\Iteration.cpp.s

CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.requires:

.PHONY : CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.requires

CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.provides: CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.requires
	$(MAKE) -f CMakeFiles\TelenavSensorFusion.dir\build.make CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.provides.build
.PHONY : CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.provides

CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.provides.build: CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj


CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj: CMakeFiles/TelenavSensorFusion.dir/flags.make
CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj: ../kalman_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj"
	C:\msys64\mingw32\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\TelenavSensorFusion.dir\kalman_filter.cpp.obj -c C:\Users\jaysh\CLionProjects\TelenavSensorFusion\kalman_filter.cpp

CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.i"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\jaysh\CLionProjects\TelenavSensorFusion\kalman_filter.cpp > CMakeFiles\TelenavSensorFusion.dir\kalman_filter.cpp.i

CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.s"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\jaysh\CLionProjects\TelenavSensorFusion\kalman_filter.cpp -o CMakeFiles\TelenavSensorFusion.dir\kalman_filter.cpp.s

CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.requires:

.PHONY : CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.requires

CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.provides: CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.requires
	$(MAKE) -f CMakeFiles\TelenavSensorFusion.dir\build.make CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.provides.build
.PHONY : CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.provides

CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.provides.build: CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj


CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj: CMakeFiles/TelenavSensorFusion.dir/flags.make
CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj: ../particle_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj"
	C:\msys64\mingw32\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\TelenavSensorFusion.dir\particle_filter.cpp.obj -c C:\Users\jaysh\CLionProjects\TelenavSensorFusion\particle_filter.cpp

CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.i"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\jaysh\CLionProjects\TelenavSensorFusion\particle_filter.cpp > CMakeFiles\TelenavSensorFusion.dir\particle_filter.cpp.i

CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.s"
	C:\msys64\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\jaysh\CLionProjects\TelenavSensorFusion\particle_filter.cpp -o CMakeFiles\TelenavSensorFusion.dir\particle_filter.cpp.s

CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.requires:

.PHONY : CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.requires

CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.provides: CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.requires
	$(MAKE) -f CMakeFiles\TelenavSensorFusion.dir\build.make CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.provides.build
.PHONY : CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.provides

CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.provides.build: CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj


# Object files for target TelenavSensorFusion
TelenavSensorFusion_OBJECTS = \
"CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj" \
"CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj" \
"CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj" \
"CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj" \
"CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj"

# External object files for target TelenavSensorFusion
TelenavSensorFusion_EXTERNAL_OBJECTS =

TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/build.make
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/linklibs.rsp
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/objects1.rsp
TelenavSensorFusion.exe: CMakeFiles/TelenavSensorFusion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable TelenavSensorFusion.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\TelenavSensorFusion.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TelenavSensorFusion.dir/build: TelenavSensorFusion.exe

.PHONY : CMakeFiles/TelenavSensorFusion.dir/build

CMakeFiles/TelenavSensorFusion.dir/requires: CMakeFiles/TelenavSensorFusion.dir/main.cpp.obj.requires
CMakeFiles/TelenavSensorFusion.dir/requires: CMakeFiles/TelenavSensorFusion.dir/tools.cpp.obj.requires
CMakeFiles/TelenavSensorFusion.dir/requires: CMakeFiles/TelenavSensorFusion.dir/Iteration.cpp.obj.requires
CMakeFiles/TelenavSensorFusion.dir/requires: CMakeFiles/TelenavSensorFusion.dir/kalman_filter.cpp.obj.requires
CMakeFiles/TelenavSensorFusion.dir/requires: CMakeFiles/TelenavSensorFusion.dir/particle_filter.cpp.obj.requires

.PHONY : CMakeFiles/TelenavSensorFusion.dir/requires

CMakeFiles/TelenavSensorFusion.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\TelenavSensorFusion.dir\cmake_clean.cmake
.PHONY : CMakeFiles/TelenavSensorFusion.dir/clean

CMakeFiles/TelenavSensorFusion.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\jaysh\CLionProjects\TelenavSensorFusion C:\Users\jaysh\CLionProjects\TelenavSensorFusion C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug C:\Users\jaysh\CLionProjects\TelenavSensorFusion\cmake-build-debug\CMakeFiles\TelenavSensorFusion.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TelenavSensorFusion.dir/depend

