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
CMAKE_SOURCE_DIR = /home/sun234/racing_work/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun234/racing_work/build

# Include any dependencies generated for this target.
include dspace_connect/CMakeFiles/vehicleStatus.dir/depend.make

# Include the progress variables for this target.
include dspace_connect/CMakeFiles/vehicleStatus.dir/progress.make

# Include the compile flags for this target's objects.
include dspace_connect/CMakeFiles/vehicleStatus.dir/flags.make

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o: dspace_connect/CMakeFiles/vehicleStatus.dir/flags.make
dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o: /home/sun234/racing_work/src/dspace_connect/src/vehicle_status.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sun234/racing_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o -c /home/sun234/racing_work/src/dspace_connect/src/vehicle_status.cpp

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.i"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sun234/racing_work/src/dspace_connect/src/vehicle_status.cpp > CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.i

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.s"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sun234/racing_work/src/dspace_connect/src/vehicle_status.cpp -o CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.s

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.requires:

.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.requires

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.provides: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.requires
	$(MAKE) -f dspace_connect/CMakeFiles/vehicleStatus.dir/build.make dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.provides.build
.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.provides

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.provides.build: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o


dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o: dspace_connect/CMakeFiles/vehicleStatus.dir/flags.make
dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o: /home/sun234/racing_work/src/dspace_connect/src/vehicle_status_core_drive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sun234/racing_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o -c /home/sun234/racing_work/src/dspace_connect/src/vehicle_status_core_drive.cpp

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.i"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sun234/racing_work/src/dspace_connect/src/vehicle_status_core_drive.cpp > CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.i

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.s"
	cd /home/sun234/racing_work/build/dspace_connect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sun234/racing_work/src/dspace_connect/src/vehicle_status_core_drive.cpp -o CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.s

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.requires:

.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.requires

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.provides: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.requires
	$(MAKE) -f dspace_connect/CMakeFiles/vehicleStatus.dir/build.make dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.provides.build
.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.provides

dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.provides.build: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o


# Object files for target vehicleStatus
vehicleStatus_OBJECTS = \
"CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o" \
"CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o"

# External object files for target vehicleStatus
vehicleStatus_EXTERNAL_OBJECTS =

/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: dspace_connect/CMakeFiles/vehicleStatus.dir/build.make
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libserial.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libtf.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libtf2_ros.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libactionlib.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libmessage_filters.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libroscpp.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libtf2.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/librosconsole.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_geometry.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_linear_algebra.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_formatters.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_exceptions.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_errors.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libecl_type_traits.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/librostime.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /opt/ros/melodic/lib/libcpp_common.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus: dspace_connect/CMakeFiles/vehicleStatus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sun234/racing_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus"
	cd /home/sun234/racing_work/build/dspace_connect && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vehicleStatus.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dspace_connect/CMakeFiles/vehicleStatus.dir/build: /home/sun234/racing_work/devel/lib/dspace_connect/vehicleStatus

.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/build

dspace_connect/CMakeFiles/vehicleStatus.dir/requires: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status.cpp.o.requires
dspace_connect/CMakeFiles/vehicleStatus.dir/requires: dspace_connect/CMakeFiles/vehicleStatus.dir/src/vehicle_status_core_drive.cpp.o.requires

.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/requires

dspace_connect/CMakeFiles/vehicleStatus.dir/clean:
	cd /home/sun234/racing_work/build/dspace_connect && $(CMAKE_COMMAND) -P CMakeFiles/vehicleStatus.dir/cmake_clean.cmake
.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/clean

dspace_connect/CMakeFiles/vehicleStatus.dir/depend:
	cd /home/sun234/racing_work/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun234/racing_work/src /home/sun234/racing_work/src/dspace_connect /home/sun234/racing_work/build /home/sun234/racing_work/build/dspace_connect /home/sun234/racing_work/build/dspace_connect/CMakeFiles/vehicleStatus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dspace_connect/CMakeFiles/vehicleStatus.dir/depend

