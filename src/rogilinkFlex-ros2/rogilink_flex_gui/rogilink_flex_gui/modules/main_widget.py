import os
import signal
import sys

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from rogilink_flex_gui.modules.console import QConsole
from rogilink_flex_gui.modules.reception_widget import QReceptionWidget
from rogilink_flex_gui.modules.style_sheet import QSS_STYLE_SHEET
from rogilink_flex_gui.modules.transmission_widget import QTransmissionWidget


class MainWidget(QWidget):
    def __init__(self, config_path: str):
        super().__init__()
        self.config_path = config_path
        self.init_ui()
    
    
    def init_ui(self):
        self.setWindowTitle("PyQt5")

        self.root_layout = QVBoxLayout()

        # 接続できてないときに赤いバーを表示
        self.connection_status = QLabel()
        self.connection_status.setText("Unconnected")
        self.connection_status.setStyleSheet("background-color: #ff3333")
        self.connection_status.setFixedHeight(20)
        self.connection_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.root_layout.addWidget(self.connection_status)
        
        
        self.main_layout = QHBoxLayout()

        # 左側のlayout
        self.console_layout = QVBoxLayout()

        # console
        self.console = QConsole()
        self.console_layout.addWidget(self.console)

        self.main_layout.addLayout(self.console_layout)

        # topic管理
        self.topic_layout = QVBoxLayout()

        # 受信
        self.reception_widget = QReceptionWidget(self.config_path)
        self.topic_layout.addWidget(self.reception_widget)

        # 送信
        self.transmission_widget = QTransmissionWidget(self.config_path)
        self.topic_layout.addWidget(self.transmission_widget)

        self.main_layout.addLayout(self.topic_layout)

        self.root_layout.addLayout(self.main_layout)
        self.setLayout(self.root_layout)

    def set_connection_status(self, is_connected: bool, message: str):
        if is_connected:
            self.connection_status.setStyleSheet("background-color: #66ff66")
            self.connection_status.setText(f"Connected: {message}")
        else:
            self.connection_status.setStyleSheet("background-color: #ff3333")
            self.connection_status.setText(f"Unconnected: {message}")

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL) #Ctrl+Cで終了
    os.environ["QT_QPA_PLATFORM"] = "xcb" # 参照 : https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System/issues/288
    os.environ["QT_LOGGING_RULES"] = "qt.pointer.*=false" #マウスポインターのログを出さない

    app = QApplication(sys.argv)
    widget = MainWidget()
    widget.show()
    sys.exit(app.exec())
    
if __name__ == '__main__':
    main()

