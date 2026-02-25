import serial
import rclpy
from rclpy.node import Node
import threading
import datetime
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from nav2_msgs.srv import ClearEntireCostmap

from imrc_conpanel.serial_resolver import *
from imrc_messages.msg import ConpanelLedControl
from imrc_messages.msg import ConpanelBuzzerControl
from imrc_messages.msg import GeneralCommand

gSerialReceive = ""
ggetRes = ""

"""
GET
G,0,0,0,0,0,0,0,0,0,0,0,0,


L4 ON/OFF
L -> 0,1,2,3,4,5,6,7


BL5 -> 5 beep 繰り返し
BL0 -> 停止
BS4 -> 4 beep 1回だけ


S,0 -> 0~7が押されたとき
x 0.0
y 0.0
z 1.0
w 0.0
"""

SERIAL_TIMEOUT = datetime.timedelta(seconds=2.0)

class ControlPanel(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # self.declare_parameter('port', '/dev/ttyACM0')
        # self.port_path = self.get_parameter('port').get_parameter_value().string_value
        conpanel_pi_sn = "92ABB4951A04E814"
        self.port_path = get_tty_by_serial(conpanel_pi_sn)
        self.get_logger().info(f'接続開始ポート: {self.port_path}')

        self.uart_utils = UartUtils(self.port_path)
        self.logger = self.get_logger()
        self.uart_utils.get_logger(self.logger)

        self.led_sub = self.create_subscription(ConpanelLedControl,'/conpanel_led', self.led_sub_callback, 10)
        self.bz_sub = self.create_subscription(ConpanelBuzzerControl,'/conpanel_bz', self.bz_sub_callback, 10)
        
        self.conpanel_miss_ball_pub = self.create_publisher(String, '/conpanel_miss_ball', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.robot_command_pub = self.create_publisher(GeneralCommand, "/robot_command", 10)

        self.start()
        self.timer_loop = self.create_timer(0.2, self.loop)
        
        self.uart_utils.port_open()
        thread_serialRead = threading.Thread(target=self.uart_utils.receiveLine_daemon, daemon=True)
        thread_parseReceived = threading.Thread(target=self.parseReceived, daemon=True)
        thread_serialRead.start()
        thread_parseReceived.start()

        self.update_duration = 0.5
        self.button_updater = self.create_timer(self.update_duration, self.button_update)

        self.button_states = [0, 0, 0, 0, 0, 0, 0, 0]
        self.button_external_states = [0, 0, 0, 0]
        self.button_external_states_pre = [0, 0, 0, 0]

        self.logger.info("Control Panel Node initialized.")
    
    def start(self):
        self.mode_operation = "MANUAL"
        self.mode_ready = "STAND BY"
        self.mode_operation_pub = self.create_publisher(String, 'mode_operation', 10)
        self.mode_ready_pub = self.create_publisher(String, 'mode_ready', 10)
        
    
    def loop(self):
        self.mode_update()
    
    def mode_update(self):
        if(self.button_external_states[0] == 0):
            self.mode_operation = "MANUAL"
        else:
            self.mode_operation = "AUTO"
        
        if(self.button_external_states[1] == 0):
            self.mode_ready = "STAND BY"
        else:
            self.mode_ready = "GO"
        
        mode_operation_str = String()
        mode_operation_str.data = self.mode_operation
        self.mode_operation_pub.publish(mode_operation_str)

        mode_ready_str = String()
        mode_ready_str.data = self.mode_ready
        self.mode_ready_pub.publish(mode_ready_str)
        

        
    
    def led_sub_callback(self, msg):
        sendBuffer = "L"
        sendBuffer += str(msg.led_index)
        sendBuffer += " "
        if(msg.led_state == True):
            sendBuffer += "ON"
        else:
            sendBuffer += "OFF"
        
        sendBuffer += "\n"

        self.uart_utils.send(sendBuffer)
        self.logger.info("Send UART \"{0}\"".format(sendBuffer.strip()))
    
    def bz_sub_callback(self, msg):
        sendBuffer = "B"
        if(msg.isloop == True):
            sendBuffer += "L"
        else:
            sendBuffer += "S"

        sendBuffer += str(msg.count)
        sendBuffer += "\n"

        self.uart_utils.send(sendBuffer)
        self.logger.info("Send UART \"{0}\"".format(sendBuffer.strip()))

    def button_update(self):
        global ggetRes
        self.uart_utils.send("GET\n")

        started = datetime.datetime.now()
        while(ggetRes == ""):
            if(datetime.datetime.now() - started >= SERIAL_TIMEOUT):
                # タイムアウト　なんらからのエラーハンドリングをしよう
                return
        
        data = ggetRes.split(',')
        for i in range(8):
            self.button_states[i] = int(data[1 + i])
        for i in range(4):
            self.button_external_states[i] = int(data[9 + i])
        
        ggetRes = ""

        for i in range(4):
            if(self.button_external_states_pre[i] != self.button_external_states[i]):
                self.processExternalButtonCommand(i)
            self.button_external_states_pre[i] = self.button_external_states[i]

    def processButtonCommand(self, buttonNumber):
        self.logger.info("Button {0} has pressed".format(buttonNumber))
        if(buttonNumber == 0):
            str = String()
            str.data = "Red"
            self.conpanel_miss_ball_pub.publish(str)
        elif(buttonNumber == 1):
            str = String()
            str.data = "Blue"
            self.conpanel_miss_ball_pub.publish(str)
        elif(buttonNumber == 2):
            str = String()
            str.data = "Yellow"
            self.conpanel_miss_ball_pub.publish(str)
        elif(buttonNumber == 3):
            str = String()
            str.data = "OK"
            self.conpanel_miss_ball_pub.publish(str)
        elif(buttonNumber == 7):
            str = String()
            str.data = "NG"
            self.conpanel_miss_ball_pub.publish(str)

        elif(buttonNumber == 4):
            self.initialize_imu()
            # self.publish_initialpose()
        
        elif(buttonNumber == 5):
            self.reset_costmap()
        
        elif(buttonNumber == 6):
            cli = self.create_client(Empty, "/conpanel_reset_miss_ball", 10)
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('/conpanel_reset_miss_ball service not available, waiting again...')
            req = Empty()
            cli.call(req)
                

    def processExternalButtonCommand(self, buttonNumber):
        if(self.button_external_states_pre[buttonNumber] == 0 and self.button_external_states[buttonNumber] == 1):
            self.logger.info("External Button {0} has pressed".format(buttonNumber))
            if(buttonNumber == 1):
                gc = GeneralCommand()
                gc.target = "calibration"
                gc.param = 0
                self.robot_command_pub.publish(gc)
        else:
            self.logger.info("External Button {0} has released".format(buttonNumber))

    def __del__(self):
        self.uart_utils.port_close()
    
    def reset_costmap(self):
        g_cli = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        l_cli = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        while not g_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Global cost map reset service not available, waiting again...')
        while not l_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Local cost map reset service not available, waiting again...')
        req = ClearEntireCostmap.Request()
        g_future = g_cli.call(req)
        l_future = l_cli.call(req)

    def initialize_imu(self):
        # ros2 service call /reset_posture std_srvs/srv/Trigger {}
        self.imu_cli = self.create_client(Trigger, "reset_posture")

        while not self.imu_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IMU reset service not available, waiting again...')
        
        imu_req = Trigger.Request()
        future = self.imu_cli.call_async(imu_req)
        future.add_done_callback(self.reset_costmap)

    
    def publish_initialpose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.z = 1.0
        initial_pose.pose.pose.orientation.w = 0.0

        self.initial_pose_pub.publish(initial_pose)

    
    def parseReceived(self):
        global gSerialReceive
        global ggetRes
        
        while(1):
            if(gSerialReceive != ""):
                data = gSerialReceive.split(',')

                if(data[0] == "G"):
                    ggetRes = gSerialReceive
                elif(data[0] == "S"):
                    self.processButtonCommand(int(data[1]))
                    self.button_states[int(data[1])] = 1
                pass

                gSerialReceive = ''


class UartUtils():
    def __init__(self, port):
        self.uart_port_num = port
        # self.uart_port_num = "/dev/ttyUSB0"
        self.uart_baud_rate = 115200
        self.logger = None

    def get_logger(self, logger):
        self.logger = logger
    
    def port_open(self):
        while True:
            try:
                self.ser = serial.Serial(self.uart_port_num, self.uart_baud_rate, timeout=1)
                self.ser.readline()
                
                break
            
            except serial.serialutil.SerialException:
                self.logger.error("Serial port not found, retrying...")
        
        self.logger.info("Seial port connected!")
    
    def send(self, data):
        self.ser.write(data.encode('utf-8'))
    
    def receiveLine_daemon(self):
        global gSerialReceive
        while(1):
            if self.ser !='':
                # data = self.ser.read(1)
                data = self.ser.readline()
                data = data.strip()
                data = data.decode('utf-8')
                gSerialReceive = data

    
    def port_close(self):
        try:
            self.ser.close()
        except:
            pass


def main(args = None):
    rclpy.init(args=args)
    uart_bridge = ControlPanel()
    rclpy.spin(uart_bridge)

    uart_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

