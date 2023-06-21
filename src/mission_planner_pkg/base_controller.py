#!/usr/bin/env python
import time
import rospy
import smach
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    SetMode,
    StreamRate
)

class BaseController():
    """BaseController is used to call the services from mavlink using mavros.
    
    Keywords arguments:
    services_timeout -- the max time to check if the services are running (default=60 seconds).
    """

    def __init__(self, services_timeout: float = 60) -> None:
        self.__service_arming = None
        self.__service_takeoff = None
        self.__service_setmode = None
        self.__service_land = None
        self.__service_streamrate = None

        self.__service_arming_proxy = None
        self.__service_takeoff_proxy = None
        self.__service_setmode_proxy = None
        self.__service_land_proxy = None
        self.__service_streamrate_proxy = None

        self.__custom_modes = None
        self.__pub_info = rospy.Publisher('base_controller_info', String, queue_size=1)

        self.__read_parameters()
        self.__init_services()

        self.__check_for_services(services_timeout)
        self.__service_streamrate_proxy(stream_id=0, message_rate=10, on_off=True)

    def __read_parameters(self) -> None:
        """Read the services names from base_controller config file."""
        self.__service_arming = rospy.get_param("services/arming")
        self.__service_takeoff = rospy.get_param("services/takeoff")
        self.__service_setmode = rospy.get_param("services/setmode")
        self.__service_land = rospy.get_param("services/land")
        self.__service_streamrate = rospy.get_param("services/set_stream_rate")
        self.__custom_modes = rospy.get_param("custom_modes")

    def __init_services(self) -> None:
        """Start the ros service proxy for each service."""
        self.__service_arming_proxy = rospy.ServiceProxy(self.__service_arming, CommandBool)
        self.__service_takeoff_proxy = rospy.ServiceProxy(self.__service_takeoff, CommandTOL)
        self.__service_setmode_proxy = rospy.ServiceProxy(self.__service_setmode, SetMode)
        self.__service_land_proxy = rospy.ServiceProxy(self.__service_land, CommandTOL) 
        self.__service_streamrate_proxy = rospy.ServiceProxy(self.__service_streamrate, StreamRate)

    def __check_for_services(self, services_timeout: float) -> None:
        """Check if all the services are running."""
        try:
            rospy.wait_for_service(self.__service_arming, timeout=services_timeout)
            rospy.wait_for_service(self.__service_takeoff, timeout=services_timeout)
            rospy.wait_for_service(self.__service_setmode, timeout=services_timeout)
            rospy.wait_for_service(self.__service_land, timeout=services_timeout)
            rospy.wait_for_service(self.__service_streamrate, timeout=services_timeout)
         except rospy.ROSException as ros_exception:
            raise rospy.ROSException from ros_exception

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
            time.sleep(2)
            self.set_custom_mode("POSHOLD")
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

    def land(self, min_pitch: float = 0.0, yaw: float = 0.0, latitude: float = 0.0, \
                longitude: float = 0.0, altitude: float = 0.0) -> bool:
        """This method call the land service with mavros
        
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
            response = self.__service_land_proxy(min_pitch, yaw, latitude, longitude, altitude)
            return response.success
        except rospy.ServiceException as service_exception:
            raise rospy.ServiceException from service_exception


                    
