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
CMAKE_SOURCE_DIR = /home/pi/ClearPathMotor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/ClearPathMotor

# Include any dependencies generated for this target.
include CMakeFiles/LibMotors.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LibMotors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LibMotors.dir/flags.make

CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o: src/LibMotors/src/motors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o -c /home/pi/ClearPathMotor/src/LibMotors/src/motors.cpp

CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibMotors/src/motors.cpp > CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.i

CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibMotors/src/motors.cpp -o CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.s

CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o: src/LibMotors/src/clearPathMotors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o -c /home/pi/ClearPathMotor/src/LibMotors/src/clearPathMotors.cpp

CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibMotors/src/clearPathMotors.cpp > CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.i

CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibMotors/src/clearPathMotors.cpp -o CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.s

CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o: src/LibINI/src/dictionary.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o -c /home/pi/ClearPathMotor/src/LibINI/src/dictionary.cpp

CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibINI/src/dictionary.cpp > CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.i

CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibINI/src/dictionary.cpp -o CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.s

CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o: src/LibINI/src/iniparser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o -c /home/pi/ClearPathMotor/src/LibINI/src/iniparser.cpp

CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibINI/src/iniparser.cpp > CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.i

CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibINI/src/iniparser.cpp -o CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.s

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o: src/LibLinuxOS/src/tekEventsLinux.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o -c /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekEventsLinux.cpp

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekEventsLinux.cpp > CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.i

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekEventsLinux.cpp -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.s

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o: src/LibLinuxOS/src/tekThreadsLinux.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o -c /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekThreadsLinux.cpp

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekThreadsLinux.cpp > CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.i

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibLinuxOS/src/tekThreadsLinux.cpp -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.s

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o: src/LibLinuxOS/src/version.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o -c /home/pi/ClearPathMotor/src/LibLinuxOS/src/version.cpp

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibLinuxOS/src/version.cpp > CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.i

CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibLinuxOS/src/version.cpp -o CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.s

CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o: src/LibXML/src/ErrCodeStr.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o -c /home/pi/ClearPathMotor/src/LibXML/src/ErrCodeStr.cpp

CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibXML/src/ErrCodeStr.cpp > CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.i

CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibXML/src/ErrCodeStr.cpp -o CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.s

CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o: src/LibXML/src/pugixml.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o -c /home/pi/ClearPathMotor/src/LibXML/src/pugixml.cpp

CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/LibXML/src/pugixml.cpp > CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.i

CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/LibXML/src/pugixml.cpp -o CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o: src/sFoundation/src/converterLib.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/converterLib.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/converterLib.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/converterLib.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o: src/sFoundation/src/cpmAPI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/cpmAPI.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/cpmAPI.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/cpmAPI.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o: src/sFoundation/src/cpmClassImpl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/cpmClassImpl.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/cpmClassImpl.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/cpmClassImpl.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o: src/sFoundation/src/iscAPI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/iscAPI.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/iscAPI.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/iscAPI.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o: src/sFoundation/src/lnkAccessCommon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/lnkAccessCommon.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/lnkAccessCommon.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/lnkAccessCommon.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o: src/sFoundation/src/meridianNet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/meridianNet.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/meridianNet.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/meridianNet.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o: src/sFoundation/src/netCmdAPI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/netCmdAPI.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/netCmdAPI.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/netCmdAPI.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o: src/sFoundation/src/netCoreFmt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/netCoreFmt.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/netCoreFmt.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/netCoreFmt.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o: src/sFoundation/src/SerialEx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/SerialEx.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/SerialEx.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/SerialEx.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o: src/sFoundation/src/sysClassImpl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src/sysClassImpl.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src/sysClassImpl.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src/sysClassImpl.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o: src/sFoundation/src-linux/lnkAccessLinux.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src-linux/lnkAccessLinux.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src-linux/lnkAccessLinux.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src-linux/lnkAccessLinux.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.s

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o: CMakeFiles/LibMotors.dir/flags.make
CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o: src/sFoundation/src-linux/SerialLinux.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Building CXX object CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o -c /home/pi/ClearPathMotor/src/sFoundation/src-linux/SerialLinux.cpp

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ClearPathMotor/src/sFoundation/src-linux/SerialLinux.cpp > CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.i

CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ClearPathMotor/src/sFoundation/src-linux/SerialLinux.cpp -o CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.s

# Object files for target LibMotors
LibMotors_OBJECTS = \
"CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o" \
"CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o" \
"CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o"

# External object files for target LibMotors
LibMotors_EXTERNAL_OBJECTS =

libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibMotors/src/motors.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibMotors/src/clearPathMotors.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibINI/src/dictionary.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibINI/src/iniparser.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekEventsLinux.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/tekThreadsLinux.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibLinuxOS/src/version.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibXML/src/ErrCodeStr.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/LibXML/src/pugixml.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/converterLib.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmAPI.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/cpmClassImpl.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/iscAPI.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/lnkAccessCommon.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/meridianNet.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/netCmdAPI.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/netCoreFmt.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/SerialEx.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src/sysClassImpl.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/lnkAccessLinux.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/src/sFoundation/src-linux/SerialLinux.cpp.o
libLibMotors.a: CMakeFiles/LibMotors.dir/build.make
libLibMotors.a: CMakeFiles/LibMotors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/ClearPathMotor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_22) "Linking CXX static library libLibMotors.a"
	$(CMAKE_COMMAND) -P CMakeFiles/LibMotors.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LibMotors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LibMotors.dir/build: libLibMotors.a

.PHONY : CMakeFiles/LibMotors.dir/build

CMakeFiles/LibMotors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LibMotors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LibMotors.dir/clean

CMakeFiles/LibMotors.dir/depend:
	cd /home/pi/ClearPathMotor && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/ClearPathMotor /home/pi/ClearPathMotor /home/pi/ClearPathMotor /home/pi/ClearPathMotor /home/pi/ClearPathMotor/CMakeFiles/LibMotors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LibMotors.dir/depend

