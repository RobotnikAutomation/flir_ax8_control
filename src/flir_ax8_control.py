#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rcomponent.rcomponent import *

# Insert here general imports:
from flir.flir import Flir
from urllib2 import HTTPError

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse


class FlirAx8Control(RComponent):
    """
    Provides ros services to control and configure the camera Flir AX8
    """

    def __init__(self):
        
        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.address = "192.168.0.185"
        self.address = rospy.get_param('~ip_address', self.address)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher

        # Subscriber
        
        # Service
        self.visual_mode_server = rospy.Service('~set_visual_mode', Trigger, self.set_visual_mode_cb)
        self.thermal_mode_server = rospy.Service('~set_thermal_mode', Trigger, self.set_thermal_mode_cb)
        self.msx_mode_server = rospy.Service('~set_msx_mode', Trigger, self.set_msx_mode_cb)

        return 0

    def init_state(self):
        self.status = String()

        self.flir = Flir(baseURL='http://'+self.address+'/')

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def set_visual_mode_cb(self, req):
        response = TriggerResponse()
        response.success = True

        msg = ""
        try:
            self.flir.setVisualMode()
            msg = "Mode set to 'visual' correctly."
            rospy.loginfo("%s::set_visual_mode_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            msg = str(error)
            rospy.logerr("%s::set_visual_mode_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response

    def set_msx_mode_cb(self, req):
        response = TriggerResponse()
        response.success = True

        msg = ""
        try:
            self.flir.setMSXMode()
            msg = "Mode set to 'msx' correctly."
            rospy.loginfo("%s::set_msx_mode_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            msg = str(error)
            rospy.logerr("%s::set_visual_mode_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response

    def set_thermal_mode_cb(self, req):
        response = TriggerResponse()
        response.success = True

        msg = ""
        try:
            self.flir.setIRMode()
            msg = "Mode set to 'thermal' correctly."
            rospy.loginfo("%s::set_thermal_mode_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            msg = str(error)
            rospy.logerr("%s::set_thermal_mode_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response
