# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/pi/teknic_motor_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/teknic_motor_controller

# Include any dependencies generated for this target.
include CMakeFiles/Example-GPIO.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Example-GPIO.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Example-GPIO.dir/flags.make

CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o: CMakeFiles/Example-GPIO.dir/flags.make
CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o: src/SDK_Examples/Example-GPIO/Example-GPIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/teknic_motor_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o -c /home/pi/teknic_motor_controller/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp

CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/teknic_motor_controller/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp > CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.i

CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/teknic_motor_controller/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp -o CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.s

# Object files for target Example-GPIO
Example__GPIO_OBJECTS = \
"CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o"

# External object files for target Example-GPIO
Example__GPIO_EXTERNAL_OBJECTS =

Example-GPIO: CMakeFiles/Example-GPIO.dir/src/SDK_Examples/Example-GPIO/Example-GPIO.cpp.o
Example-GPIO: CMakeFiles/Example-GPIO.dir/build.make
Example-GPIO: libsFoundation.a
Example-GPIO: libLibINI.a
Example-GPIO: libLibLinuxOS.a
Example-GPIO: libLibXML.a
Example-GPIO: CMakeFiles/Example-GPIO.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/teknic_motor_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Example-GPIO"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Example-GPIO.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Example-GPIO.dir/build: Example-GPIO

.PHONY : CMakeFiles/Example-GPIO.dir/build

CMakeFiles/Example-GPIO.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Example-GPIO.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Example-GPIO.dir/clean

CMakeFiles/Example-GPIO.dir/depend:
	cd /home/pi/teknic_motor_controller && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/teknic_motor_controller /home/pi/teknic_motor_controller /home/pi/teknic_motor_controller /home/pi/teknic_motor_controller /home/pi/teknic_motor_controller/CMakeFiles/Example-GPIO.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Example-GPIO.dir/depend
