#!/usr/bin/env python
import time
import rospy

import numpy as np

from sensor_msgs.msg import Range

class BaseVision:
    def __init__(self) -> None:
        
        self.__init_subscribers()

        self.__passed_direction = None #salvar quando ler o QR
        self.__current_direction = None



    def __init_subscribers(self) -> None:
        """Start the subscribers"""
        pass

    def get_angle_to_turne(self) -> float:
        """
        
        """
        if (self.__passed_direction == "N" and self.__current_direction == "N") or \
           (self.__passed_direction == "S" and self.__current_direction == "S") or \
           (self.__passed_direction == "E" and self.__current_direction == "E") or \
           (self.__passed_direction == "W" and self.__current_direction == "W"):

           return 0.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "E") or \
             (self.__passed_direction == "S" and self.__current_direction == "W") or \
             (self.__passed_direction == "E" and self.__current_direction == "S") or \
             (self.__passed_direction == "W" and self.__current_direction == "N"):
           
           return 90.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "W") or \
             (self.__passed_direction == "S" and self.__current_direction == "E") or \
             (self.__passed_direction == "E" and self.__current_direction == "N") or \
             (self.__passed_direction == "W" and self.__current_direction == "S"):
           
           return -90.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "S") or \
             (self.__passed_direction == "S" and self.__current_direction == "N") or \
             (self.__passed_direction == "E" and self.__current_direction == "W") or \
             (self.__passed_direction == "W" and self.__current_direction == "E"):
           
           return 180.0
        
    def is_qr_detected(self):
        pass