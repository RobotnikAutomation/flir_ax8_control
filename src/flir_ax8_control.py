#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rcomponent.rcomponent import *

# Insert here general imports:
from flir.flir import Flir
from urllib2 import HTTPError

# Insert here msg and srv imports:
from flir_ax8_msgs.msg import Alarm, Alarms, Box, Boxes
from flir_ax8_msgs.srv import GetSpotTemperature, GetSpotTemperatureResponse, \
                                SetPalette, SetPaletteResponse

from std_msgs.msg import String

from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse


class FlirAx8Control(RComponent):
    """
    Provides ros services to control and configure the camera Flir AX8
    """

    def __init__(self):
        
        RComponent.__init__(self)

        self.boxes_msg = Boxes()
        self.t_boxes_updater = threading.Timer(0.2, self.update_boxes)
        self.t_boxes_updater.start()

        self.alarms_msg = Alarms()
        self.t_alarms_updater = threading.Timer(1.0, self.update_alarms)
        self.t_alarms_updater.start()



    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.address = "192.168.0.185"
        self.address = rospy.get_param('~ip_address', self.address)
        
        self.max_num_boxes = 6
        self.max_num_boxes = rospy.get_param('~max_num_boxes', self.max_num_boxes)

        self.max_temp_alarms = 5
        self.max_temp_alarms = rospy.get_param('~max_temp_alarms', self.max_temp_alarms)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.boxes_pub = rospy.Publisher('~boxes', Boxes, queue_size=1)
        self.alarms_pub = rospy.Publisher('~alarms', Alarms, queue_size=1)

        # Subscriber
        
        # Service
        self.visual_mode_srv = rospy.Service('~set_visual_mode', Trigger, self.set_visual_mode_cb)
        self.thermal_mode_srv = rospy.Service('~set_thermal_mode', Trigger, self.set_thermal_mode_cb)
        self.msx_mode_srv = rospy.Service('~set_msx_mode', Trigger, self.set_msx_mode_cb)
        self.show_overlay_srv = rospy.Service('~show_overlay', SetBool, self.show_overlay_cb)
        self.show_overlay_srv = rospy.Service('~turn_light', SetBool, self.turn_light_cb)
        self.get_spot_temp_srv = rospy.Service('~get_spot_temperature', GetSpotTemperature, self.get_spot_temp_cb)
        self.set_palette_srv = rospy.Service('~set_palette', SetPalette, self.set_palette_cb)

        return 0

    def init_state(self):
        self.status = String()

        self.flir = Flir(baseURL='http://'+self.address+'/', 
                        maxBoxes=self.max_num_boxes, 
                        maxAlarms=self.max_temp_alarms)

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
        # Cancels current timers
        self.t_boxes_updater.cancel()

        self.boxes_pub.unregister()
        self.alarms_pub.unregister()

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def ros_publish(self):
        self.boxes_pub.publish(self.boxes_msg)
        self.alarms_pub.publish(self.alarms_msg)

        RComponent.ros_publish(self)

    def set_visual_mode_cb(self, req):
        response = TriggerResponse()
        response.success = True

        msg = ""
        try:
            self.flir.setVisualMode()
            msg = "Mode set to 'visual' correctly."
            rospy.loginfo("%s::set_visual_mode_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            response.success = False
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
            response.success = False
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
            response.success = False
            msg = str(error)
            rospy.logerr("%s::set_thermal_mode_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response
    
    def show_overlay_cb(self, req):
        response = SetBoolResponse()
        response.success = True

        msg = ""
        try:
            self.flir.showOverlay(req.data)
            msg = "Overlay set correctly."
            rospy.loginfo("%s::show_overlay_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            response.success = False
            msg = str(error)
            rospy.logerr("%s::show_overlay_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response

    def turn_light_cb(self, req):
        response = SetBoolResponse()
        response.success = True

        msg = ""
        try:
            self.flir.light(req.data)
            msg = "Overlay set correctly."
            rospy.loginfo("%s::turn_light_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            response.success = False
            msg = str(error)
            rospy.logerr("%s::turn_light_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response

    def get_spot_temp_cb(self, req):
        response = GetSpotTemperatureResponse()
        response.success = True

        msg = ""
        try:
            response.temperature = self.flir.getSpotTemperatureValue(req.spot)
            msg = "Temperature for spot " + str(req.spot) + " is " + str(response.temperature)
            rospy.loginfo("%s::get_spot_temp_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            response.success = False
            msg = str(error)
            rospy.logerr("%s::get_spot_temp_cb:: %s" % (self._node_name, error))

        return response

    def set_palette_cb(self, req):
        response = SetPaletteResponse()
        response.success = True

        msg = ""
        try:
            self.flir.setPalette(req.name+ ".pal")
            msg = "Palette set to " + req.name

            rospy.loginfo("%s::set_palette_cb:: %s" % (self._node_name, msg))

        except HTTPError as error:
            response.success = False
            msg = str(error)
            rospy.logerr("%s::set_palette_cb:: %s" % (self._node_name, error))

        response.message = msg
        return response

    def update_boxes(self):
        if self._initialized == False:
            return

        boxes = self.flir.getBoxes()
        boxes_msg = Boxes()
        
        for box in boxes:
            box_msg = Box()

            try:
                box_msg.boxNumber = int(box['boxNumber'])

                if (box['active'] == '"true"'):
                    box_msg.active = True
                
                box_msg.minT = float(box['minT'][1:-2])
                box_msg.maxT = float(box['maxT'][1:-2])
                box_msg.avgT = float(box['avgT'][1:-2])
                
                boxes_msg.boxes.append(box_msg)
            except KeyError as e:
                continue
        
        self.boxes_msg = boxes_msg
        self.t_boxes_updater = threading.Timer(0.2, self.update_boxes)
        self.t_boxes_updater.start()
    
    def update_alarms(self):
        if self._initialized == False:
            return

        alarms = self.flir.getAlarms()
        alarms_msg = Alarms()
        
        for alarm in alarms:
            alarm_msg = Alarm()

            try:
                alarm_msg.alarmNumber = int(alarm['alarmNumber'])
                alarm_msg.type = alarm['type'][1:-1]

                if (alarm['active'] == '"true"'):
                    alarm_msg.active = True
                
                if (alarm['trigged'] == '"true"'):
                    alarm_msg.trigged = True
                
                alarms_msg.alarms.append(alarm_msg)
            except KeyError as e:
                continue
        
        self.alarms_msg = alarms_msg
        self.t_alarms_updater = threading.Timer(1.0, self.update_alarms)
        self.t_alarms_updater.start()
            
