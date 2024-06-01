# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "rospy;geometry_msgs;mavros_msgs;roscpp;src_loc_msgs;arva_sim;roslaunch".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsrc_loc".split(';') if "-lsrc_loc" != "" else []
PROJECT_NAME = "src_loc"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
