import rclpy
from rclpy.node import Node
from src.flir.flir import Flir
from urllib.error import HTTPError, URLError
from flir_ax8_msgs.msg import Alarm, Alarms, Box, Boxes
from flir_ax8_msgs.srv import GetSpotTemperature, SetPalette
from std_srvs.srv import Trigger, SetBool

class FlirAx8Control(Node):
    """
    Provides ros services to control and configure the camera Flir AX8
    """
    def __init__(self):
        super().__init__('flir_ax8_control')

        # Parameters
        self.declare_parameter('ip_address', '192.168.0.104')
        self.declare_parameter('max_num_boxes', 6)
        self.declare_parameter('max_temp_alarms', 5)
        self.declare_parameter('request_timeout_sec', 1.0)
        self.address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.max_num_boxes = self.get_parameter('max_num_boxes').get_parameter_value().integer_value
        self.max_temp_alarms = self.get_parameter('max_temp_alarms').get_parameter_value().integer_value
        self.request_timeout_sec = self.get_parameter('request_timeout_sec').get_parameter_value().double_value

        # FLIR object
        self.flir = Flir(
            baseURL='http://' + self.address + '/',
            maxBoxes=self.max_num_boxes,
            maxAlarms=self.max_temp_alarms,
            timeout=self.request_timeout_sec,
        )

        # Publishers
        self.boxes_pub = self.create_publisher(Boxes, 'flir_ax8_control/boxes', 1)
        self.alarms_pub = self.create_publisher(Alarms, 'flir_ax8_control/alarms', 1)

        # Services
        self.visual_mode_srv = self.create_service(Trigger, 'flir_ax8_control/set_visual_mode', self.set_visual_mode_cb)
        self.thermal_mode_srv = self.create_service(Trigger, 'flir_ax8_control/set_thermal_mode', self.set_thermal_mode_cb)
        self.msx_mode_srv = self.create_service(Trigger, 'flir_ax8_control/set_msx_mode', self.set_msx_mode_cb)
        self.show_overlay_srv = self.create_service(SetBool, 'flir_ax8_control/show_overlay', self.show_overlay_cb)
        self.turn_light_srv = self.create_service(SetBool, 'flir_ax8_control/turn_light', self.turn_light_cb)
        self.get_spot_temp_srv = self.create_service(GetSpotTemperature, 'flir_ax8_control/get_spot_temperature', self.get_spot_temp_cb)
        self.set_palette_srv = self.create_service(SetPalette, 'flir_ax8_control/set_palette', self.set_palette_cb)

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
        try:
            boxes = self.flir.getBoxes()
        except (HTTPError, URLError, OSError) as error:
            self.get_logger().warning(f"FLIR connection error while reading boxes: {error}")
            return

        boxes_msg = Boxes()
        for box in boxes:
            box_msg = Box()
            try:
                box_msg.box_number = int(box['boxNumber'])
                box_msg.active = (box.get('active', '"false"') == '"true"')

                if box_msg.active:
                    box_msg.min_t = float(box['minT'][1:-2])
                    box_msg.max_t = float(box['maxT'][1:-2])
                    box_msg.avg_t = float(box['avgT'][1:-2])
                else:
                    # Keep inactive boxes visible in diagnostics output.
                    box_msg.min_t = float('nan')
                    box_msg.max_t = float('nan')
                    box_msg.avg_t = float('nan')

                boxes_msg.boxes.append(box_msg)
            except (KeyError, ValueError) as error:
                self.get_logger().debug(f"Skipping malformed box payload: {error}")
                continue
        self.boxes_msg = boxes_msg
        self.boxes_pub.publish(self.boxes_msg)

    def update_alarms(self):
        try:
            alarms = self.flir.getAlarms()
        except (HTTPError, URLError, OSError) as error:
            self.get_logger().warning(f"FLIR connection error while reading alarms: {error}")
            return

        alarms_msg = Alarms()
        for alarm in alarms:
            alarm_msg = Alarm()
            try:
                alarm_msg.alarm_number = int(alarm['alarmNumber'])
                alarm_msg.type = alarm['type'][1:-1]
                alarm_msg.active = (alarm['active'] == '"true"')
                alarm_msg.trigged = (alarm['trigged'] == '"true"')
                alarms_msg.alarms.append(alarm_msg)
            except KeyError as e:
                self.get_logger().warn(str(e))
                continue
        self.alarms_msg = alarms_msg
        self.alarms_pub.publish(self.alarms_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlirAx8Control()
    node.get_logger().info(f'{node.get_name()}: starting')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
