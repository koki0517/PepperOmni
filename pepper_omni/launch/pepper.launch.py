# Copyright 2022 eSOL Co.,Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler, ExecuteProcess)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

import glob
import re
import time
import datetime

def generate_launch_description():

  usb2canfd = Node(
		package="uec_core",
		executable="usb2canfd",
		output="screen",
		parameters=[
			{"last_id1": 2},
			{"last_id2": 4},
		]
  )

  pepper_omni = Node(
    package="pepper_omni",
    executable="dxl_omni",
    name="dxl_omni",
    output="screen",
    parameters=[
      {"id_list": [1,2,3,4]},
      {"hz": 100.0},
    ],
  )

  joy_pub = Node(
    package="pepper_omni",
    executable="joy_pub",
    name="joy_pub",
    output="screen",
  )

  joy_linux = Node(
    package="joy",
    executable="joy_node",
    name="joy",
    output="screen",
    parameters=[],
  )

  return LaunchDescription([
    # usb2canfd,
    pepper_omni,
    joy_pub,
    joy_linux,
    ])