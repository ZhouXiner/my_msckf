# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/zhouxin/Documents/app/clion-2019.2.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zhouxin/Documents/app/clion-2019.2.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhouxin/Documents/my_msckf/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhouxin/Documents/my_msckf/build

# Include any dependencies generated for this target.
include msckf/CMakeFiles/track_node.dir/depend.make

# Include the progress variables for this target.
include msckf/CMakeFiles/track_node.dir/progress.make

# Include the compile flags for this target's objects.
include msckf/CMakeFiles/track_node.dir/flags.make

msckf/CMakeFiles/track_node.dir/src/track_node.cpp.o: msckf/CMakeFiles/track_node.dir/flags.make
msckf/CMakeFiles/track_node.dir/src/track_node.cpp.o: /home/zhouxin/Documents/my_msckf/src/msckf/src/track_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhouxin/Documents/my_msckf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object msckf/CMakeFiles/track_node.dir/src/track_node.cpp.o"
	cd /home/zhouxin/Documents/my_msckf/build/msckf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/track_node.dir/src/track_node.cpp.o -c /home/zhouxin/Documents/my_msckf/src/msckf/src/track_node.cpp

msckf/CMakeFiles/track_node.dir/src/track_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_node.dir/src/track_node.cpp.i"
	cd /home/zhouxin/Documents/my_msckf/build/msckf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhouxin/Documents/my_msckf/src/msckf/src/track_node.cpp > CMakeFiles/track_node.dir/src/track_node.cpp.i

msckf/CMakeFiles/track_node.dir/src/track_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_node.dir/src/track_node.cpp.s"
	cd /home/zhouxin/Documents/my_msckf/build/msckf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhouxin/Documents/my_msckf/src/msckf/src/track_node.cpp -o CMakeFiles/track_node.dir/src/track_node.cpp.s

# Object files for target track_node
track_node_OBJECTS = \
"CMakeFiles/track_node.dir/src/track_node.cpp.o"

# External object files for target track_node
track_node_EXTERNAL_OBJECTS =

/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: msckf/CMakeFiles/track_node.dir/src/track_node.cpp.o
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: msckf/CMakeFiles/track_node.dir/build.make
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libcv_bridge.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/librandom_numbers.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libtf.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libtf2_ros.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libactionlib.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libtf2.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libroscpp.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/librosconsole.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/librostime.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /opt/ros/melodic/lib/libcpp_common.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_stitching.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_superres.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_videostab.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_aruco.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_bgsegm.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_bioinspired.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_ccalib.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_dpm.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_face.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_freetype.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_fuzzy.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_hdf.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_hfs.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_img_hash.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_line_descriptor.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_optflow.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_reg.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_rgbd.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_saliency.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_stereo.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_structured_light.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_surface_matching.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_tracking.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_xfeatures2d.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_ximgproc.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_xobjdetect.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_xphoto.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_shape.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_viz.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_video.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_datasets.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_plot.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_text.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_dnn.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_highgui.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_ml.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_videoio.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_imgcodecs.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_objdetect.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_calib3d.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_features2d.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_flann.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_photo.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_imgproc.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: /usr/local/lib/libopencv_core.so.3.4.7
/home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node: msckf/CMakeFiles/track_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhouxin/Documents/my_msckf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node"
	cd /home/zhouxin/Documents/my_msckf/build/msckf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/track_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
msckf/CMakeFiles/track_node.dir/build: /home/zhouxin/Documents/my_msckf/devel/lib/msckf/track_node

.PHONY : msckf/CMakeFiles/track_node.dir/build

msckf/CMakeFiles/track_node.dir/clean:
	cd /home/zhouxin/Documents/my_msckf/build/msckf && $(CMAKE_COMMAND) -P CMakeFiles/track_node.dir/cmake_clean.cmake
.PHONY : msckf/CMakeFiles/track_node.dir/clean

msckf/CMakeFiles/track_node.dir/depend:
	cd /home/zhouxin/Documents/my_msckf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhouxin/Documents/my_msckf/src /home/zhouxin/Documents/my_msckf/src/msckf /home/zhouxin/Documents/my_msckf/build /home/zhouxin/Documents/my_msckf/build/msckf /home/zhouxin/Documents/my_msckf/build/msckf/CMakeFiles/track_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : msckf/CMakeFiles/track_node.dir/depend

