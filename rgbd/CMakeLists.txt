cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

##opencv?

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_boost_directories()

################################################################################
# create libraries

#rosbuild_add_library (ccny_rgbd_features src/structures/rgbd_frame.cpp
#                                         src/features/feature_detector.cpp
#                                         src/features/orb_detector.cpp
#                                         src/features/gft_detector.cpp
#                                         src/features/canny_detector.cpp
#                                         src/features/surf_detector.cpp
#                                         src/util.cpp)

#target_link_libraries(ccny_rgbd_features ${OPENCV_LIBRARIES})  


################################################################################
# create feature_viewer_node executable

#rosbuild_add_executable(feature_viewer_node 
#                          src/node/feature_viewer_node.cpp
#                          src/features/feature_viewer.cpp)
                                            
#target_link_libraries(feature_viewer_node ccny_rgbd_features)
#target_link_libraries(feature_viewer_node ccny_gicp) 
#target_link_libraries(feature_viewer_node ${OPENCV_LIBRARIES})   

################################################################################

rosbuild_add_executable(vo_icp_node 
                          src/node/vo_icp_node.cpp
                          src/structures/rgbd_frame2.cpp
                          src/structures/rgbd_keyframe.cpp
                          src/features/feature_detector.cpp
                          src/features/orb_detector.cpp
                          src/features/gft_detector.cpp
                          src/features/klt_detector.cpp
                          src/features/surf_detector.cpp
                          src/util.cpp
                          src/registration/vo_icp.cpp
                          src/registration/icp_kd.cpp
                          src/registration/feature_history.cpp)

target_link_libraries (vo_icp_node ${OPENCV_LIBRARIES}) 
target_link_libraries (vo_icp_node boost_signals)

#
#rosbuild_add_library(vo_icp_nodelet 
#                          src/nodelet/vo_icp_nodelet.cpp
#                          src/structures/rgbd_frame2.cpp
#                          src/features/feature_detector.cpp
#                          #src/features/orb_detector.cpp
#                          #src/features/gft_detector.cpp
#                          #src/features/canny_detector.cpp
#                          src/features/surf_detector.cpp
#                          src/util.cpp
#                          src/registration/vo_icp.cpp
#                          src/registration/icp_kd.cpp
#                          src/registration/feature_history.cpp)

#target_link_libraries (vo_icp_nodelet ${OPENCV_LIBRARIES}) 
#target_link_libraries (vo_icp_nodelet boost_signals)


###################################################################

#rosbuild_add_executable(sparse_tracker_node 
#                          src/node/sparse_tracker_node.cpp
#                          src/registration/sparse_tracker.cpp
#                          src/registration/icp_kd.cpp
#                          src/registration/feature_history.cpp
#                          src/registration/loop_solver.cpp
#                          src/structures/edge.cpp)
                       
#target_link_libraries (sparse_tracker_node ccny_rgbd_features)  
#target_link_libraries (sparse_tracker_node ccny_gicp)               

#add toro lib

#add_library(toro3d STATIC IMPORTED)
#rosbuild_find_ros_package(ccny_toro)
#set_property(TARGET toro3d PROPERTY IMPORTED_LOCATION ${ccny_toro_PACKAGE_PATH}/lib/libtoro3d.a)
#target_link_libraries(sparse_tracker_node toro3d) # will link using full path

################################################################################

#rosbuild_add_executable(sparse_tracker_f_node 
#                        src/node/sparse_tracker_f_node.cpp
#                        src/registration/sparse_tracker_f.cpp
#                        src/structures/edge.cpp
#                        src/registration/icp_kd.cpp
#                        src/registration/feature_history.cpp)
                      
#target_link_libraries(sparse_tracker_f_node ccny_rgbd_features)
#target_link_libraries (sparse_tracker_f_node boost_program_options)

#add_library(toro3d STATIC IMPORTED)

#rosbuild_find_ros_package(ccny_toro)

#set_property(TARGET toro3d PROPERTY IMPORTED_LOCATION ${ccny_toro_PACKAGE_PATH}/lib/libtoro3d.a)

#target_link_libraries(sparse_tracker_f_node toro3d) # will link using full path

################################################################################

#rosbuild_add_executable(dense_tracker_node 
#                          src/node/dense_tracker_node.cpp
#                          src/registration/dense_tracker.cpp
#                          src/util.cpp)

#target_link_libraries (dense_tracker_node ccny_gicp) 

################################################################################

#rosbuild_add_executable(dense_tracker_gicp_node 
#                          src/node/dense_tracker_gicp_node.cpp
#                          src/registration/dense_tracker_gicp.cpp
#                          src/util.cpp)

#target_link_libraries (dense_tracker_gicp_node ccny_gicp) 


