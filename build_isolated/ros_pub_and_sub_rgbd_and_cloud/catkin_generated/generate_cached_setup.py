# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/isro/ros_catkin_ws/install_isolated/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/isro/ros_catkin_ws/install_isolated/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/isro/isro_ws/devel_isolated/realsense2_description;/home/isro/isro_ws/devel_isolated/realsense2_camera;/home/isro/isro_ws/devel_isolated/move_base_msgs;/home/isro/isro_ws/devel_isolated/map_msgs;/home/isro/isro_ws/devel_isolated/libmavconn;/home/isro/isro_ws/devel_isolated/geographic_info;/home/isro/isro_ws/devel_isolated/ddynamic_reconfigure;/home/isro/ros_catkin_ws/devel_isolated/rqt_rviz;/home/isro/ros_catkin_ws/install_isolated'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/isro/isro_ws/devel_isolated/ros_pub_and_sub_rgbd_and_cloud/env.sh')

output_filename = '/home/isro/isro_ws/build_isolated/ros_pub_and_sub_rgbd_and_cloud/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
