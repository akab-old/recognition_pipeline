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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/valerio/Scrivania/recognition_pipeline

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/valerio/Scrivania/recognition_pipeline/build

# Include any dependencies generated for this target.
include CMakeFiles/recognition_pip.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/recognition_pip.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/recognition_pip.dir/flags.make

CMakeFiles/recognition_pip.dir/main.cpp.o: CMakeFiles/recognition_pip.dir/flags.make
CMakeFiles/recognition_pip.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/valerio/Scrivania/recognition_pipeline/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/recognition_pip.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/recognition_pip.dir/main.cpp.o -c /home/valerio/Scrivania/recognition_pipeline/main.cpp

CMakeFiles/recognition_pip.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/recognition_pip.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/valerio/Scrivania/recognition_pipeline/main.cpp > CMakeFiles/recognition_pip.dir/main.cpp.i

CMakeFiles/recognition_pip.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/recognition_pip.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/valerio/Scrivania/recognition_pipeline/main.cpp -o CMakeFiles/recognition_pip.dir/main.cpp.s

CMakeFiles/recognition_pip.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/recognition_pip.dir/main.cpp.o.requires

CMakeFiles/recognition_pip.dir/main.cpp.o.provides: CMakeFiles/recognition_pip.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/recognition_pip.dir/build.make CMakeFiles/recognition_pip.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/recognition_pip.dir/main.cpp.o.provides

CMakeFiles/recognition_pip.dir/main.cpp.o.provides.build: CMakeFiles/recognition_pip.dir/main.cpp.o

# Object files for target recognition_pip
recognition_pip_OBJECTS = \
"CMakeFiles/recognition_pip.dir/main.cpp.o"

# External object files for target recognition_pip
recognition_pip_EXTERNAL_OBJECTS =

recognition_pip: CMakeFiles/recognition_pip.dir/main.cpp.o
recognition_pip: CMakeFiles/recognition_pip.dir/build.make
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_system.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_thread.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libpthread.so
recognition_pip: /usr/local/lib/libpcl_common.so
recognition_pip: /usr/local/lib/libpcl_octree.so
recognition_pip: /usr/lib/libOpenNI.so
recognition_pip: /usr/lib/libOpenNI2.so
recognition_pip: /usr/lib/libvtkCommon.so.5.8.0
recognition_pip: /usr/lib/libvtkFiltering.so.5.8.0
recognition_pip: /usr/lib/libvtkImaging.so.5.8.0
recognition_pip: /usr/lib/libvtkGraphics.so.5.8.0
recognition_pip: /usr/lib/libvtkGenericFiltering.so.5.8.0
recognition_pip: /usr/lib/libvtkIO.so.5.8.0
recognition_pip: /usr/lib/libvtkRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkVolumeRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkHybrid.so.5.8.0
recognition_pip: /usr/lib/libvtkWidgets.so.5.8.0
recognition_pip: /usr/lib/libvtkParallel.so.5.8.0
recognition_pip: /usr/lib/libvtkInfovis.so.5.8.0
recognition_pip: /usr/lib/libvtkGeovis.so.5.8.0
recognition_pip: /usr/lib/libvtkViews.so.5.8.0
recognition_pip: /usr/lib/libvtkCharts.so.5.8.0
recognition_pip: /usr/local/lib/libpcl_io.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
recognition_pip: /usr/local/lib/libpcl_kdtree.so
recognition_pip: /usr/local/lib/libpcl_search.so
recognition_pip: /usr/local/lib/libpcl_sample_consensus.so
recognition_pip: /usr/local/lib/libpcl_filters.so
recognition_pip: /usr/local/lib/libpcl_features.so
recognition_pip: /usr/local/lib/libpcl_ml.so
recognition_pip: /usr/local/lib/libpcl_segmentation.so
recognition_pip: /usr/local/lib/libpcl_visualization.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libqhull.so
recognition_pip: /usr/local/lib/libpcl_surface.so
recognition_pip: /usr/local/lib/libpcl_registration.so
recognition_pip: /usr/local/lib/libpcl_keypoints.so
recognition_pip: /usr/local/lib/libpcl_tracking.so
recognition_pip: /usr/local/lib/libpcl_recognition.so
recognition_pip: /usr/local/lib/libpcl_stereo.so
recognition_pip: /usr/local/lib/libpcl_outofcore.so
recognition_pip: /usr/local/lib/libpcl_people.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_system.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_thread.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libpthread.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libqhull.so
recognition_pip: /usr/lib/libOpenNI.so
recognition_pip: /usr/lib/libOpenNI2.so
recognition_pip: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
recognition_pip: /usr/lib/libvtkCommon.so.5.8.0
recognition_pip: /usr/lib/libvtkFiltering.so.5.8.0
recognition_pip: /usr/lib/libvtkImaging.so.5.8.0
recognition_pip: /usr/lib/libvtkGraphics.so.5.8.0
recognition_pip: /usr/lib/libvtkGenericFiltering.so.5.8.0
recognition_pip: /usr/lib/libvtkIO.so.5.8.0
recognition_pip: /usr/lib/libvtkRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkVolumeRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkHybrid.so.5.8.0
recognition_pip: /usr/lib/libvtkWidgets.so.5.8.0
recognition_pip: /usr/lib/libvtkParallel.so.5.8.0
recognition_pip: /usr/lib/libvtkInfovis.so.5.8.0
recognition_pip: /usr/lib/libvtkGeovis.so.5.8.0
recognition_pip: /usr/lib/libvtkViews.so.5.8.0
recognition_pip: /usr/lib/libvtkCharts.so.5.8.0
recognition_pip: /usr/local/lib/libpcl_common.so
recognition_pip: /usr/local/lib/libpcl_octree.so
recognition_pip: /usr/local/lib/libpcl_io.so
recognition_pip: /usr/local/lib/libpcl_kdtree.so
recognition_pip: /usr/local/lib/libpcl_search.so
recognition_pip: /usr/local/lib/libpcl_sample_consensus.so
recognition_pip: /usr/local/lib/libpcl_filters.so
recognition_pip: /usr/local/lib/libpcl_features.so
recognition_pip: /usr/local/lib/libpcl_ml.so
recognition_pip: /usr/local/lib/libpcl_segmentation.so
recognition_pip: /usr/local/lib/libpcl_visualization.so
recognition_pip: /usr/local/lib/libpcl_surface.so
recognition_pip: /usr/local/lib/libpcl_registration.so
recognition_pip: /usr/local/lib/libpcl_keypoints.so
recognition_pip: /usr/local/lib/libpcl_tracking.so
recognition_pip: /usr/local/lib/libpcl_recognition.so
recognition_pip: /usr/local/lib/libpcl_stereo.so
recognition_pip: /usr/local/lib/libpcl_outofcore.so
recognition_pip: /usr/local/lib/libpcl_people.so
recognition_pip: /usr/lib/libvtkViews.so.5.8.0
recognition_pip: /usr/lib/libvtkInfovis.so.5.8.0
recognition_pip: /usr/lib/libvtkWidgets.so.5.8.0
recognition_pip: /usr/lib/libvtkVolumeRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkHybrid.so.5.8.0
recognition_pip: /usr/lib/libvtkParallel.so.5.8.0
recognition_pip: /usr/lib/libvtkRendering.so.5.8.0
recognition_pip: /usr/lib/libvtkImaging.so.5.8.0
recognition_pip: /usr/lib/libvtkGraphics.so.5.8.0
recognition_pip: /usr/lib/libvtkIO.so.5.8.0
recognition_pip: /usr/lib/libvtkFiltering.so.5.8.0
recognition_pip: /usr/lib/libvtkCommon.so.5.8.0
recognition_pip: /usr/lib/libvtksys.so.5.8.0
recognition_pip: CMakeFiles/recognition_pip.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable recognition_pip"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/recognition_pip.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/recognition_pip.dir/build: recognition_pip
.PHONY : CMakeFiles/recognition_pip.dir/build

CMakeFiles/recognition_pip.dir/requires: CMakeFiles/recognition_pip.dir/main.cpp.o.requires
.PHONY : CMakeFiles/recognition_pip.dir/requires

CMakeFiles/recognition_pip.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/recognition_pip.dir/cmake_clean.cmake
.PHONY : CMakeFiles/recognition_pip.dir/clean

CMakeFiles/recognition_pip.dir/depend:
	cd /home/valerio/Scrivania/recognition_pipeline/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/valerio/Scrivania/recognition_pipeline /home/valerio/Scrivania/recognition_pipeline /home/valerio/Scrivania/recognition_pipeline/build /home/valerio/Scrivania/recognition_pipeline/build /home/valerio/Scrivania/recognition_pipeline/build/CMakeFiles/recognition_pip.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/recognition_pip.dir/depend

