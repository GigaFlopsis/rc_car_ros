#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
from rospy_websocker_client import WebsocketROSClient as ros_ws

from geometry_msgs.msg import  PoseStamped
from mavros_msgs.srv import SetModeRequest, CommandBoolRequest

from sensor_msgs.msg import BatteryState
from drone_msgs.msg import Diagnostics
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from sensor_msgs.msg import NavSatFix
from drone_msgs.msg import Goal

class DroneConnect(QObject):

    diag_signals = pyqtSignal()
    def __init__(self, websocket_ip, port=9090, name =''):
        super(DroneConnect, self).__init__()

        self.diagnostics = Diagnostics()
        # sub to server
        self.name = name
        self.ip = websocket_ip
        self.port = port
        self.ws = ros_ws.ws_client(self.ip, self.port)
        self.ws.subscribe('/mavros/local_position/pose', PoseStamped(), name+"/local/pose")
        self.ws.subscribe('/geo/local_pose', PoseStamped(), name+"/geo/local_pose")
        self.ws.subscribe('/diagnostics', Diagnostics(), name+"/diagnostics")
        self.ws.subscribe('/mavros/global_position/global', NavSatFix(), name+"/global/pose")

        # Ros subscribe
        self.sub_diagnoctics = rospy.Subscriber(name+"/diagnostics", Diagnostics, self.clb_diag)
        self.sub_goal = rospy.Subscriber(name+"/geo/goal_pose", Goal, self.goal_clb_global)
        self.sub_goal = rospy.Subscriber(name+"/local/goal_pose", Goal, self.goal_clb_local)


    def connect(self):
        self.ws.connect()

    def disconnect(self):
        self.sub_diagnoctics.unregister()
        self.sub_goal.unregister()
        self.ws.disconnect()

    def __del__(self):
        self.ws.__del__()

    def goal_clb_local(self, data):
        """
        Get goal pose
        :type data: PoseStamped
        :return:
        """
        self.ws.publish("/goal_pose", data)


    def goal_clb_global(self, data):
        """
        Get goal pose
        :type data: PoseStamped
        :return:
        """
        self.ws.publish("/geo/goal_pose", data)

    def clb_diag(self, data):
        """
        Get battery data of drone
        :type data: BatteryState
        :return:
        """
        self.diagnostics = data
        self.diag_signals.emit()

    def is_active(self):
        return self.ws.is_connected()

    def set_mode(self,new_mode):
        """
        Функция отправки нового режима работы автопилота.

        @param new_mode: новый режим работы атопилота
        @type new_mode: String
        """
        print("set mode")
        mode = SetModeRequest()
        mode.custom_mode = new_mode

        if self.diagnostics.mode != new_mode:
            self.ws.call_service("mavros/set_mode", mode)

    def set_origin(self, topic, data):
        print("set origin")
        self.ws.publish(topic, data)

    def arm(self):
        arm_state = CommandBoolRequest()
        arm_state.value = True
        self.ws.call_service("mavros/cmd/arming", arm_state)

    def disarm(self):
        arm_state = CommandBoolRequest()
        arm_state.value = False
        self.ws.call_service("mavros/cmd/arming", arm_state)



