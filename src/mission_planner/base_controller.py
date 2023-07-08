#!/usr/bin/env python
import time
import rospy

import numpy as np
from tf.transformations import euler_from_quaternion
 
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    SetMode,
    StreamRate,
    SetMavFrame
)
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped

class BaseController():
    """BaseController is used to call the services from mavlink using mavros.
    
    Keywords arguments:
    services_timeout -- the max time to check if the services are running (default=60 seconds).
    """

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
    
    def __init__(self, services_timeout: float = 60) -> None:
        self.state = None
        self.rangefinder = None
        self.current_position = None
        self.current_turne = 0

        self.__read_parameters()
        self.__init_services()
        self.__init_publishers()
        self.__init_subscribers()

        self.__check_for_services(services_timeout)
        assert self.__enable_mavros_topics()

    def __read_parameters(self) -> None:
        """Read the services names from base_controller config file"""

        # Services
        self.__service_arming = rospy.get_param("services/arming")
        self.__service_takeoff = rospy.get_param("services/takeoff")
        self.__service_setmode = rospy.get_param("services/setmode")
        self.__service_land = rospy.get_param("services/land")
        self.__service_streamrate = rospy.get_param("services/stream_rate")
        self.__service_set_frame = rospy.get_param("services/set_frame")

        # Publishers
        self.__setpoint_local = rospy.get_param("publishers/setpoint_position_local")
        self.__setpoint_cmd_vel = rospy.get_param("publishers/setpoint_velocity_cmd_vel")

        # Subscribers
        self.__state = rospy.get_param("subscribers/state")
        self.__rangefinder = rospy.get_param("subscribers/rangefinder")
        self.__local_position_pose = rospy.get_param("subscribers/local_position_pose")

        # Common
        self.__custom_modes = rospy.get_param("custom_modes")

    def __init_services(self) -> None:
        """Start the ros service proxy for each service"""
        self.__service_arming_proxy = rospy.ServiceProxy(self.__service_arming, CommandBool)
        self.__service_takeoff_proxy = rospy.ServiceProxy(self.__service_takeoff, CommandTOL)
        self.__service_setmode_proxy = rospy.ServiceProxy(self.__service_setmode, SetMode)
        self.__service_land_proxy = rospy.ServiceProxy(self.__service_land, CommandTOL)
        self.__service_streamrate_proxy = rospy.ServiceProxy(self.__service_streamrate, StreamRate)
        self.__service_set_frame_proxy = rospy.ServiceProxy(self.__service_set_frame, SetMavFrame)

    def __init_publishers(self) -> None:
        """Start the publishers"""
        self.__publisher_setpoint_local = rospy.Publisher(self.__setpoint_local, \
                                                          PoseStamped, \
                                                          queue_size=1)


        self.__publisher_setpoint_cmd_vel = rospy.Publisher(self.__setpoint_cmd_vel, \
                                                          PoseStamped, \
                                                          queue_size=1)
        
    def __init_subscribers(self) -> None:
        """Start the subscribers"""
        rospy.Subscriber(self.__state, State, \
                         self.__state_callback)
        rospy.Subscriber(self.__rangefinder, Range, \
                         self.__rangefinder_callback)
        rospy.Subscriber(self.__local_position_pose, PoseStamped, \
                         self.__local_position_pose_callback)

    def __state_callback(self, msg: State) -> None:
        """The callback method to verify the flight state"""
        self.state = msg

    def __rangefinder_callback(self, msg: Range) -> None:
        """The callback to get the rangefinder range"""
        self.rangefinder = msg

    def __local_position_pose_callback(self, msg: PoseStamped) -> None:
        """The callback to verify the drone position"""
        self.current_position = msg

    def __check_for_services(self, services_timeout: float) -> None:
        """Check if all the services are running"""
        try:
            rospy.wait_for_service(self.__service_arming, timeout=services_timeout)
            rospy.wait_for_service(self.__service_takeoff, timeout=services_timeout)
            rospy.wait_for_service(self.__service_setmode, timeout=services_timeout)
            rospy.wait_for_service(self.__service_land, timeout=services_timeout)
            rospy.wait_for_service(self.__service_streamrate, timeout=services_timeout)
            rospy.wait_for_service(self.__service_set_frame, timeout=services_timeout)

        except rospy.ROSException as ros_exception:
            raise rospy.ROSException from ros_exception

    def __enable_mavros_topics(self) -> bool:
        """Enable the topics of mavros"""
        
        try:
            self.__service_streamrate_proxy(stream_id=0, message_rate=10, on_off=True)
            return True
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception

    def arm(self) -> bool:
        """This method call the arming service to arm the drone 
        
        Returns:
        response.success -- returns true if the service worked correctly.
        """
        try:
            response = self.__service_arming_proxy(True)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception

    def disarm(self) -> bool:
        """This method call the arming service to disarm the drone
        
        Returns:
        response.success -- returns true if the service worked correctly.
        """
        try:
            response = self.__service_arming_proxy(False)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception

    def takeoff(self, min_pitch: float = 0.0, yaw: float = 0.0, latitude: float = 0.0, \
                longitude: float = 0.0, altitude: float = 0.0) -> bool:
        """This method call the takeoff service

        Keywords arguments:
        min_pitch   -- The minimum allowed pitch angle during takeoff (default set as 0).
        yaw         -- The desired direction angle during takeoff (usually set as 0 to maintain the current direction).
        latitude    -- the latitude of the desired takeoff position (usually set as 0 to use the current position).
        longitude   -- the longitude of the desired takeoff position (usually set as 0 to use the current position).
        altitude    -- the desired altitude for UAV takeoff (default set as 0).

        Returns:
        response.success -- returns true if the service worked correctly.
        """
        try:
            self.set_custom_mode("LOITER")
            time.sleep(2)
            response = self.__service_takeoff_proxy(min_pitch, yaw, latitude, longitude, altitude)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception

    def set_custom_mode(self, custom_mode: str = ""):
        """This method set a custom mode to UAV

        Keywords arguments:
        custom_mode -- The custom mode string that specifies the desired mode of the UAV, allowing you to set a specific behavior or flight mode defined in the flight controller or autopilot software.

        Possible custom modes:
        - [STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, POSITION, LAND, OF_LOITER, DRIFT, SPORT, FLIP, AUTOTUNE, POSHOLD, BRAKE, THROW, AVOID_ADSB, GUIDED_NOGPS]

        Returns:
        response.mode_sent -- returns true if the service worked correctly.
        """
        assert custom_mode in self.__custom_modes

        try:
            response = self.__service_setmode_proxy(0, custom_mode)
            return response.mode_sent
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception
    
    def set_custom_frame(self, custom_frame: int = 1):
        """This method set a custom mode to UAV

        Keywords arguments:
        custom_mode -- The custom mode string that specifies the desired mode of the UAV, allowing you to set a specific behavior or flight mode defined in the flight controller or autopilot software.

        Possible custom modes:
        - [STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, POSITION, LAND, OF_LOITER, DRIFT, SPORT, FLIP, AUTOTUNE, POSHOLD, BRAKE, THROW, AVOID_ADSB, GUIDED_NOGPS]

        Returns:
        response.mode_sent -- returns true if the service worked correctly.
        """
        try:
            response = self.__service_set_frame_proxy(custom_frame)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception
    

    def land(self, min_pitch: float = 0.0, yaw: float = 0.0, latitude: float = 0.0, \
                longitude: float = 0.0, altitude: float = 0.0) -> bool:
        """This method call the land service with mavros
        
        Keywords arguments:
        min_pitch   -- The minimum allowed pitch angle during takeoff (default set as 0).
        yaw         -- The desired direction angle during takeoff (usually set as 0 to maintain the current direction).
        latitude    -- The latitude of the desired takeoff position (usually set as 0 to use the current position).
        longitude   -- The longitude of the desired takeoff position (usually set as 0 to use the current position).
        altitude    -- The desired altitude for UAV takeoff (default set as 0).

        Returns:
        response.success -- returns true if the service worked correctly.
        """
        try:
            response = self.__service_land_proxy(min_pitch, yaw, latitude, longitude, altitude)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception
        

    def set_position(self, position_x: float = 0.0, position_y: float = 0.0, \
                     position_z: float = 0.0) -> bool:
        """This method publish a PoseStamped message in setpoint_position/local
        
        Keywords arguments:
        position_x  -- The target position in x
        position_y  -- The target position in y
        position_z  -- The target position in z

        Returns:
        True
        """
        try:
            self.set_custom_mode("GUIDED")

            pose = PoseStamped()
            pose.pose.position.x = position_x
            pose.pose.position.y = position_y
            pose.pose.position.z = position_z

            self.__publisher_setpoint_local.publish(pose)

            return True
        except rospy.ROSException as ros_exception:
            raise rospy.ROSException from ros_exception
        
    def point_from_distance_rotation(self, z : float = None, step : float = 0.1):
        """
        Convert an Euler angle to a quaternion.

        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        a = np.radians(self.current_turne)
        dx = np.cos(a) * step
        dy = np.sin(a) * step
        z = self.get_current_position()[0]

        self.set_position(dx, dy, z)

        return [dx, dy, z]
        
    def set_velocity(self, x: float = 0.0, z: float = 0.0) -> bool:
        """This method publish a PoseStamped message in setpoint_position/local
        
        Keywords arguments:
        position_x  -- The target position in x
        position_z  -- The target position in z

        Returns:
        True
        """
        try:
            self.set_custom_mode("GUIDED")

            vel = TwistStamped()
            vel.twist.linear.x = x
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = z

            vel.twist.angular.x = 0.0
            vel.twist.angular.y = 0.0
            vel.twist.angular.z = 0.0

            self.__publisher_setpoint_cmd_vel.publish(vel)

            return True
        except rospy.ROSException as ros_exception:
            raise rospy.ROSException from ros_exception
        
    def set_turne(self, turne: float = 0.0) -> bool:
        """This method publish a PoseStamped message in setpoint_position/local
        
        Keywords arguments:
        turne  -- The target orientation in z

        Returns:
        True
        """
        try:
            self.set_custom_mode("GUIDED")
            
            self.current_turne += turne
            p_x, p_y, p_z = self.__turne_position
            x, y, z, w = self.quaternion_from_euler(0, 0, np.radians(turne))

            pose = PoseStamped()
            pose.pose.position.x = p_x
            pose.pose.position.y = p_y
            pose.pose.position.z = p_z
            pose.pose.orientation.x = x
            pose.pose.orientation.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w

            self.__publisher_setpoint_local.publish(pose)

            return self.__turne_position
        except rospy.ROSException as ros_exception:
            raise rospy.ROSException from ros_exception
        
    def save_position_to_turne(self) -> bool:
        """Save current position of the drone
        
        Returns:
        __turne_position    -- The current position of the drone
        """
        self.__turne_position = self.get_current_position()
        return True

    def get_current_position(self) -> list:
        """Get current position of the drone
        
        Returns:
        current_position    -- The current position of the drone
        """
        p_x = self.current_position.pose.position.x
        p_y = self.current_position.pose.position.y
        p_z = self.current_position.pose.position.z
        return [p_x, p_y, p_z]
    
    def get_current_turne(self) -> float:
        """Get current yaw of the drone
        
        Returns:
        current_turne    -- The current turne of the drone in degrees
        """
        x, y, z, w = self.current_position.pose.orientation.x, self.current_position.pose.orientation.y, self.current_position.pose.orientation.z, self.current_position.pose.orientation.w
        current_turne = np.degrees(euler_from_quaternion([x, y, z, w]))[2]
        return current_turne


    def get_height(self) -> float:
        """Get the current height of the drone
        
        Returns:
        current_height      -- The current height of the drone
        """
        current_height = self.rangefinder.range
        return current_height
    
    def get_mode(self) -> str:
        """Get the current flight mode of the drone
        
        Returns:
        mode      -- The current flight mode of the drone
        """
        mode = self.state.mode
        return mode
    
    def is_armed(self) -> bool:
        """Get the current armed status
        
        Returns:
        status      -- The current armed status of the drone
        """
        status = self.state.armed
        return status
    
    def is_target_height(self, target_height: float = 0.0, threshold: float = 0.0) -> bool:
        """Get the evatuation of the drone difference to target heigth with a threashold
        
        Returns:
        eval    -- The evatuation of the drone difference to target heigth
        """
        eval = True if (target_height - self.get_height()) <= threshold else False
        return eval
    
    def is_target_position(self, target_position: list = [0.0, 0.0, 0.0], threshold: float = 0.0) -> bool:
        """Get the evatuation of the drone difference to target position with a threashold
        
        Returns:
        eval    -- The evatuation of the drone difference to target position
        """
        target_position = np.array(target_position)
        current_position = np.array(self.get_current_position())
        eval = True if np.linalg.norm(target_position - current_position) <= threshold else False
        return eval
    
    def is_target_turne(self, target_turne: float = 0.0, threshold: float = 0.0) -> bool:
        """Get the evatuation of the drone difference to target turne with a threashold
        
        Returns:
        eval    -- The evatuation of the drone difference to target turne 
        """
        current_turne = self.get_current_turne()
        eval = True if target_turne - current_turne <= threshold else False
        return eval