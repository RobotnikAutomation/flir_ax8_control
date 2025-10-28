import rclpy
from rclpy.node import Node
import threading
from flir.flir import Flir
from urllib2 import HTTPError
from flir_ax8_msgs.msg import Alarm, Alarms, Box, Boxes
from flir_ax8_msgs.srv import GetSpotTemperature, GetSpotTemperatureResponse, SetPalette, SetPaletteResponse
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

class FlirAx8Control(Node):
    """
    Provides ros services to control and configure the camera Flir AX8
    """
    def __init__(self):
        super().__init__('flir_ax8_control')

        # Parameters
        self.declare_parameter('ip_address', '192.168.0.185')
        self.declare_parameter('max_num_boxes', 6)
        self.declare_parameter('max_temp_alarms', 5)
        self.address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.max_num_boxes = self.get_parameter('max_num_boxes').get_parameter_value().integer_value
        self.max_temp_alarms = self.get_parameter('max_temp_alarms').get_parameter_value().integer_value

        # FLIR object
        self.flir = Flir(baseURL='http://' + self.address + '/', maxBoxes=self.max_num_boxes, maxAlarms=self.max_temp_alarms)

        # Publishers
        self.boxes_pub = self.create_publisher(Boxes, 'boxes', 1)
        self.alarms_pub = self.create_publisher(Alarms, 'alarms', 1)

        # Services
        self.visual_mode_srv = self.create_service(Trigger, 'set_visual_mode', self.set_visual_mode_cb)
        self.thermal_mode_srv = self.create_service(Trigger, 'set_thermal_mode', self.set_thermal_mode_cb)
        self.msx_mode_srv = self.create_service(Trigger, 'set_msx_mode', self.set_msx_mode_cb)
        self.show_overlay_srv = self.create_service(SetBool, 'show_overlay', self.show_overlay_cb)
        self.turn_light_srv = self.create_service(SetBool, 'turn_light', self.turn_light_cb)
        self.get_spot_temp_srv = self.create_service(GetSpotTemperature, 'get_spot_temperature', self.get_spot_temp_cb)
        self.set_palette_srv = self.create_service(SetPalette, 'set_palette', self.set_palette_cb)

        # Timers
        self.boxes_msg = Boxes()
        self.alarms_msg = Alarms()
        self.boxes_timer = self.create_timer(0.2, self.update_boxes)
        self.alarms_timer = self.create_timer(1.0, self.update_alarms)

    def set_visual_mode_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.setVisualMode()
            msg = "Mode set to 'visual' correctly."
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def set_msx_mode_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.setMSXMode()
            msg = "Mode set to 'msx' correctly."
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def set_thermal_mode_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.setIRMode()
            msg = "Mode set to 'thermal' correctly."
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def show_overlay_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.showOverlay(request.data)
            msg = "Overlay set correctly."
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def turn_light_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.light(request.data)
            msg = "Light set correctly."
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def get_spot_temp_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            response.temperature = self.flir.getSpotTemperatureValue(request.spot)
            msg = f"Temperature for spot {request.spot} is {response.temperature}"
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def set_palette_cb(self, request, response):
        response.success = True
        msg = ""
        try:
            self.flir.setPalette(request.name + ".pal")
            msg = f"Palette set to {request.name}"
            self.get_logger().info(msg)
        except HTTPError as error:
            response.success = False
            msg = str(error)
            self.get_logger().error(msg)
        response.message = msg
        return response

    def update_boxes(self):
        boxes = self.flir.getBoxes()
        boxes_msg = Boxes()
        for box in boxes:
            box_msg = Box()
            try:
                box_msg.boxNumber = int(box['boxNumber'])
                box_msg.active = (box['active'] == '"true"')
                box_msg.minT = float(box['minT'][1:-2])
                box_msg.maxT = float(box['maxT'][1:-2])
                box_msg.avgT = float(box['avgT'][1:-2])
                boxes_msg.boxes.append(box_msg)
            except (KeyError, ValueError) as error:
                self.get_logger().warn(str(error))
                continue
        self.boxes_msg = boxes_msg
        self.boxes_pub.publish(self.boxes_msg)

    def update_alarms(self):
        alarms = self.flir.getAlarms()
        alarms_msg = Alarms()
        for alarm in alarms:
            alarm_msg = Alarm()
            try:
                alarm_msg.alarmNumber = int(alarm['alarmNumber'])
                alarm_msg.type = alarm['type'][1:-1]
                alarm_msg.active = (alarm['active'] == '"true"')
                alarm_msg.trigged = (alarm['trigged'] == '"true"')
                alarms_msg.alarms.append(alarm_msg)
            except KeyError as e:
                self.get_logger().warn(str(e))
                continue
        self.alarms_msg = alarms_msg
        self.alarms_pub.publish(self.alarms_msg)
        
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
            
            except ValueError as error:
                rospy.logwarn_throttle(2, "%s::update_boxes:: %s" % (self._node_name, error))
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
            
