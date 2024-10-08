# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /data/share/Software/cmake/cmake-3.20.0-install/bin/cmake

# The command to remove a file.
RM = /data/share/Software/cmake/cmake-3.20.0-install/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /data/suween/corner_detect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/suween/corner_detect/build

# Include any dependencies generated for this target.
include src/test/CMakeFiles/corner.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/test/CMakeFiles/corner.dir/compiler_depend.make

# Include the progress variables for this target.
include src/test/CMakeFiles/corner.dir/progress.make

# Include the compile flags for this target's objects.
include src/test/CMakeFiles/corner.dir/flags.make

src/test/CMakeFiles/corner.dir/corner.cpp.o: src/test/CMakeFiles/corner.dir/flags.make
src/test/CMakeFiles/corner.dir/corner.cpp.o: ../src/test/corner.cpp
src/test/CMakeFiles/corner.dir/corner.cpp.o: src/test/CMakeFiles/corner.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/suween/corner_detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/test/CMakeFiles/corner.dir/corner.cpp.o"
	cd /data/suween/corner_detect/build/src/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/test/CMakeFiles/corner.dir/corner.cpp.o -MF CMakeFiles/corner.dir/corner.cpp.o.d -o CMakeFiles/corner.dir/corner.cpp.o -c /data/suween/corner_detect/src/test/corner.cpp

src/test/CMakeFiles/corner.dir/corner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/corner.dir/corner.cpp.i"
	cd /data/suween/corner_detect/build/src/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/suween/corner_detect/src/test/corner.cpp > CMakeFiles/corner.dir/corner.cpp.i

src/test/CMakeFiles/corner.dir/corner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/corner.dir/corner.cpp.s"
	cd /data/suween/corner_detect/build/src/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/suween/corner_detect/src/test/corner.cpp -o CMakeFiles/corner.dir/corner.cpp.s

# Object files for target corner
corner_OBJECTS = \
"CMakeFiles/corner.dir/corner.cpp.o"

# External object files for target corner
corner_EXTERNAL_OBJECTS =

src/test/corner: src/test/CMakeFiles/corner.dir/corner.cpp.o
src/test/corner: src/test/CMakeFiles/corner.dir/build.make
src/test/corner: ../3rdparty/boost_install/lib/libboost_filesystem.so
src/test/corner: src/ethzasl_apriltag2/libethzasl_apriltag2.a
src/test/corner: src/april_detect/libapril_detect.a
src/test/corner: src/ethzasl_apriltag2/libethzasl_apriltag2.a
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
src/test/corner: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
src/test/corner: src/test/CMakeFiles/corner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/suween/corner_detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable corner"
	cd /data/suween/corner_detect/build/src/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/corner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/test/CMakeFiles/corner.dir/build: src/test/corner
.PHONY : src/test/CMakeFiles/corner.dir/build

src/test/CMakeFiles/corner.dir/clean:
	cd /data/suween/corner_detect/build/src/test && $(CMAKE_COMMAND) -P CMakeFiles/corner.dir/cmake_clean.cmake
.PHONY : src/test/CMakeFiles/corner.dir/clean

src/test/CMakeFiles/corner.dir/depend:
	cd /data/suween/corner_detect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/suween/corner_detect /data/suween/corner_detect/src/test /data/suween/corner_detect/build /data/suween/corner_detect/build/src/test /data/suween/corner_detect/build/src/test/CMakeFiles/corner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/test/CMakeFiles/corner.dir/depend

