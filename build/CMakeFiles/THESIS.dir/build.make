# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build

# Include any dependencies generated for this target.
include CMakeFiles/THESIS.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/THESIS.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/THESIS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/THESIS.dir/flags.make

CMakeFiles/THESIS.dir/src/main.cpp.o: CMakeFiles/THESIS.dir/flags.make
CMakeFiles/THESIS.dir/src/main.cpp.o: /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/src/main.cpp
CMakeFiles/THESIS.dir/src/main.cpp.o: CMakeFiles/THESIS.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/THESIS.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/THESIS.dir/src/main.cpp.o -MF CMakeFiles/THESIS.dir/src/main.cpp.o.d -o CMakeFiles/THESIS.dir/src/main.cpp.o -c /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/src/main.cpp

CMakeFiles/THESIS.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/THESIS.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/src/main.cpp > CMakeFiles/THESIS.dir/src/main.cpp.i

CMakeFiles/THESIS.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/THESIS.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/src/main.cpp -o CMakeFiles/THESIS.dir/src/main.cpp.s

# Object files for target THESIS
THESIS_OBJECTS = \
"CMakeFiles/THESIS.dir/src/main.cpp.o"

# External object files for target THESIS
THESIS_EXTERNAL_OBJECTS =

THESIS: CMakeFiles/THESIS.dir/src/main.cpp.o
THESIS: CMakeFiles/THESIS.dir/build.make
THESIS: /opt/homebrew/lib/libopencv_gapi.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_stitching.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_alphamat.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_aruco.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_bgsegm.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_bioinspired.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_ccalib.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_dnn_objdetect.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_dnn_superres.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_dpm.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_face.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_freetype.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_fuzzy.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_hfs.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_img_hash.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_intensity_transform.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_line_descriptor.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_mcc.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_quality.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_rapid.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_reg.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_rgbd.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_saliency.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_sfm.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_stereo.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_structured_light.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_superres.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_surface_matching.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_tracking.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_videostab.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_viz.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_wechat_qrcode.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_xfeatures2d.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_xobjdetect.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_xphoto.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_shape.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_highgui.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_datasets.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_plot.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_text.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_ml.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_phase_unwrapping.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_optflow.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_ximgproc.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_video.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_videoio.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_imgcodecs.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_objdetect.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_calib3d.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_dnn.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_features2d.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_flann.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_photo.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_imgproc.4.8.1.dylib
THESIS: /opt/homebrew/lib/libopencv_core.4.8.1.dylib
THESIS: CMakeFiles/THESIS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable THESIS"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/THESIS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/THESIS.dir/build: THESIS
.PHONY : CMakeFiles/THESIS.dir/build

CMakeFiles/THESIS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/THESIS.dir/cmake_clean.cmake
.PHONY : CMakeFiles/THESIS.dir/clean

CMakeFiles/THESIS.dir/depend:
	cd /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0 /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0 /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build /Users/danendracleveroananda/Documents/Kuliah/TA/Thesis2.0/build/CMakeFiles/THESIS.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/THESIS.dir/depend

