#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

__all__ = [
    'get_ros_version',
    'init', 'ok', 'shutdown', 'on_shutdown',
    'ros_timestamp',
    'get_service_request', 'get_service_response',
    'logdebug', 'loginfo', 'logwarn', 'logerr', 'logfatal'
]

import ros_compatibility.callback_groups
import ros_compatibility.exceptions
import ros_compatibility.executors
import ros_compatibility.node
import ros_compatibility.qos

from ros_compatibility.core import get_ros_version
from ros_compatibility.logging import logdebug, loginfo, logwarn, logerr, logfatal

ROS_VERSION = get_ros_version()

import ament_index_python.packages
import rclpy
from builtin_interfaces.msg import Time

_shutdown_hooks = []

def init(name, args=None):
    rclpy.init(args=args)

def ok():
    return rclpy.ok()

def shutdown():
    global _shutdown_hooks
    for h in _shutdown_hooks:
        h()
    rclpy.shutdown()

def _add_shutdown_hook(hook):
    if not callable(hook):
        raise TypeError("shutdown hook [%s] must be a function or callable object: %s"%(hook, type(hook)))
    _shutdown_hooks.append(hook)

def on_shutdown(hook):
    _add_shutdown_hook(hook)

def ros_timestamp(sec=0, nsec=0, from_sec=False):
    time = Time()
    if from_sec:
        time.sec = int(sec)
        time.nanosec = int((sec - int(sec)) * 1000000000)
    else:
        time.sec = int(sec)
        time.nanosec = int(nsec)
    return time

def get_service_request(service_type):
    return service_type.Request()

def get_service_response(service_type):
    return service_type.Response()

def get_package_share_directory(package_name):
    return ament_index_python.packages.get_package_share_directory(package_name)