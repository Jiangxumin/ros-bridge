#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""

import math

import carla_common.transforms as trans
from carla_ros_bridge.sensor import Sensor

from sensor_msgs.msg import NavSatFix
# ROS2 GPSFix
from gps_msgs.msg import GPSFix


class Gnss(Sensor):

    """
    Actor implementation details for gnss sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Gnss, self).__init__(uid=uid,
                                   name=name,
                                   parent=parent,
                                   relative_spawn_pose=relative_spawn_pose,
                                   node=node,
                                   carla_actor=carla_actor,
                                   synchronous_mode=synchronous_mode)

        self.gnss_publisher = node.new_publisher(NavSatFix,
                                                 self.get_topic_prefix(),
                                                 qos_profile=10)
        self.gps_fix_publisher = node.new_publisher(GPSFix,
                                                     '/sensing/gnss/fix',
                                                     qos_profile=10)
        self.listen()

    def destroy(self):
        super(Gnss, self).destroy()
        self.node.destroy_publisher(self.gnss_publisher)
        self.node.destroy_publisher(self.gps_fix_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_gnss_measurement):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = self.get_msg_header(timestamp=carla_gnss_measurement.timestamp)
        navsatfix_msg.latitude = carla_gnss_measurement.latitude
        navsatfix_msg.longitude = carla_gnss_measurement.longitude
        navsatfix_msg.altitude = carla_gnss_measurement.altitude
        self.gnss_publisher.publish(navsatfix_msg)

        # Publish GPSFix message
        gpsfix_msg = GPSFix()
        gpsfix_msg.header = self.get_msg_header(timestamp=carla_gnss_measurement.timestamp)
        
        # Position
        gpsfix_msg.latitude = carla_gnss_measurement.latitude
        gpsfix_msg.longitude = carla_gnss_measurement.longitude
        gpsfix_msg.altitude = carla_gnss_measurement.altitude
        
        # Status (default to GPS fix)
        gpsfix_msg.status.status = 0  # STATUS_FIX
        gpsfix_msg.status.satellites_used = 0
        gpsfix_msg.status.satellites_visible = 0
        gpsfix_msg.status.motion_source = 0
        gpsfix_msg.status.orientation_source = 0
        gpsfix_msg.status.position_source = 1  # SOURCE_GPS
        
        # Get motion and orientation from parent vehicle
        try:
            # Get vehicle transform (for orientation)
            if self.parent and hasattr(self.parent, 'carla_actor'):
                vehicle_transform = self.parent.carla_actor.get_transform()
                vehicle_velocity = self.parent.carla_actor.get_velocity()
                
                # Orientation (roll, pitch, yaw in radians from carla_rotation_to_RPY)
                roll, pitch, yaw = trans.carla_rotation_to_RPY(vehicle_transform.rotation)
                gpsfix_msg.roll = math.degrees(roll)
                gpsfix_msg.pitch = math.degrees(pitch)
                gpsfix_msg.dip = math.degrees(yaw)  # Magnetic dip angle, not available
                
                # Track (direction from north in degrees, 0-360)
                # carla_rotation_to_RPY returns yaw in radians [-π, π]
                # Convert to degrees and normalize to [0, 360)
                # Add 90 degrees clockwise rotation for coordinate system alignment
                yaw_degrees = math.degrees(yaw)
                # Normalize to [0, 360) range
                gpsfix_msg.track = ((180.0 - yaw_degrees) +270.0) % 360.0
                
                # Speed calculations
                # Horizontal speed (ground speed in meters/second)
                horizontal_speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2)
                gpsfix_msg.speed = horizontal_speed
                
                # Vertical speed (climb rate in meters/second)
                gpsfix_msg.climb = vehicle_velocity.z
            else:
                # No parent vehicle, set to default
                gpsfix_msg.track = 0.0
                gpsfix_msg.speed = 0.0
                gpsfix_msg.climb = 0.0
                gpsfix_msg.pitch = 0.0
                gpsfix_msg.roll = 0.0
                gpsfix_msg.dip = 0.0
        except (AttributeError, TypeError):
            # Fallback to default values if parent data not available
            gpsfix_msg.track = 0.0
            gpsfix_msg.speed = 0.0
            gpsfix_msg.climb = 0.0
            gpsfix_msg.pitch = 0.0
            gpsfix_msg.roll = 0.0
            gpsfix_msg.dip = 0.0
        
        # Time
        gpsfix_msg.time = 0.0
        
        # Dilution of precision (unknown values set to -1.0)
        gpsfix_msg.gdop = -1.0
        gpsfix_msg.pdop = -1.0
        gpsfix_msg.hdop = -1.0
        gpsfix_msg.vdop = -1.0
        gpsfix_msg.tdop = -1.0
        
        # Error estimates (unknown values set to 0.0)
        gpsfix_msg.err = 0.0
        gpsfix_msg.err_horz = 0.0
        gpsfix_msg.err_vert = 0.0
        gpsfix_msg.err_track = 0.0
        gpsfix_msg.err_speed = 0.0
        gpsfix_msg.err_climb = 0.0
        gpsfix_msg.err_time = 0.0
        gpsfix_msg.err_pitch = 0.0
        gpsfix_msg.err_roll = 0.0
        gpsfix_msg.err_dip = 0.0
        
        # Position covariance (unknown)
        gpsfix_msg.position_covariance = [0.0] * 9
        gpsfix_msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
        
        self.gps_fix_publisher.publish(gpsfix_msg)
