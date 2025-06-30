
import os
import signal
import sys
import time
from ctypes import *

import rclpy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from rclpy.node import Node
from rogilink_flex_gui.modules.main_widget import MainWidget
from rogilink_flex_gui.modules.style_sheet import QSS_STYLE_SHEET
from rogilink_flex_interfaces.srv import IsConnected
from rogilink_flex_lib import Publisher, Subscriber

try:
    import jsonc as json
except ImportError:
    import json



        
class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()

        # ros2のノードを保持
        self.node = node
        self.node.get_logger().info("Hello, World!")

        # parameter
        self.node.declare_parameter('config_path', f'rogilink_flex.json ')
        self.config_file_path = self.node.get_parameter('config_path').value

        # guiの初期化
        self.init_ui()

        # PublisherとSubscriberの初期化
        self.create_publisher_and_subscriber()

        # publisherの送信ボタンが押されたときに呼ばれる関数を設定
        self.gui.transmission_widget.set_send_message_callback(self.send_message)

        # rclpy.spin_onceを呼び出すためのタイマー
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.rclpy_loop)
        self.spin_timer.start(10)

        # 接続状態を確認するためのclient, timer
        self.is_connected_client = self.node.create_client(IsConnected, 'is_connected')
        self.connection_check_timer = QTimer(self)
        self.connection_check_timer.timeout.connect(self.check_connection)
        self.connection_check_timer.start(1000)

    def create_publisher_and_subscriber(self):
        with open(self.config_file_path, 'r') as f:
            self.configs = json.load(f)

        self.publisher: dict[str, Publisher] = {}
        self.subscriber: dict[str, Subscriber]  = {}

        self.transmission_configs = {}
        self.reception_configs = {}

        for topic_config in self.configs['transmission_messages']:
            topic_name = topic_config['name']
            self.transmission_configs[topic_name] = topic_config
            self.publisher[topic_name] = Publisher(self.node, topic_name, topic_config['type'])

        for topic_config in self.configs['reception_messages']:
            topic_name = topic_config['name']
            self.reception_configs[topic_name] = topic_config
            self.subscriber[topic_name] = Subscriber(self.node, topic_name, topic_config['type'], lambda msg, topic_name=topic_name: self.reception_callback(topic_name, msg))

    def reception_callback(self, topic_name, msg):
        '''受信したメッセージをコンソールに表示する'''
        if not self.gui.reception_widget.is_active_topic(topic_name):
            return
        
        topic_config = self.reception_configs[topic_name]
        if ',' not in topic_config['type']:
            msg = [msg]
        if 'format' not in topic_config:
            if 'args' not in topic_config:
                msg_text = ""
            else:
                msg_dir = {type_name.strip():value for type_name, value in zip(topic_config['args'].split(','), msg)}
                msg_text = str(msg_dir)
        else:
            msg_text = topic_config['format'] % tuple(msg)
        self.gui.console.append(f'[{time.time():.5f}] {topic_name}: {msg_text}')
        self.gui.reception_widget.topic_widgets[topic_name].last_message_label.setText(str(msg_text))

    def send_message(self, topic_name, msg):
        '''メッセージを送信する'''
        self.node.get_logger().info(f'Send message: {topic_name} {msg}')
        self.publisher[topic_name].publish(msg)

    def init_ui(self):
        self.setWindowTitle("RogilinkFlex GUI")
        self.gui = MainWidget(self.config_file_path)
        self.setCentralWidget(self.gui)

    def rclpy_loop(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def check_connection(self):
        '''接続状態を確認する'''
        if not self.is_connected_client.wait_for_service(timeout_sec=0):
            self.gui.set_connection_status(False, "rogilink_flex is not running")
            return
        
        request = IsConnected.Request()
        future = self.is_connected_client.call_async(request)
        future.add_done_callback(self.future_callback)
    
    def future_callback(self, future):
        '''接続状態を確認したときのコールバック'''
        try:
            result = future.result()
        except Exception as e:
            self.gui.set_connection_status(False, "rogilink_flex is not running")
            return
        if result.connected:
            self.gui.set_connection_status(True, str(result.device_id))
        else:
            self.gui.set_connection_status(False, "waiting for connection")
    
def main():
    rclpy.init()
    node = Node('rogilink_flex_gui')

    signal.signal(signal.SIGINT, signal.SIG_DFL) #Ctrl+Cで終了
    os.environ["QT_QPA_PLATFORM"] = "xcb" # 参照 : https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System/issues/288
    os.environ["QT_LOGGING_RULES"] = "qt.pointer.*=false" #マウスポインターのログを出さない

    app = QApplication(sys.argv)
    widget = MainWindow(node)

    widget.show() # ウィジェットを表示
    sys.exit(app.exec()) # pyqtイベントループを開始
    
if __name__ == '__main__':
    main()

