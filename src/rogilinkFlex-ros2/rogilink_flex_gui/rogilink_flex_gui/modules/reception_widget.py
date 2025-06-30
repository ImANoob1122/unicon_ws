import os
import typing

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

try:
    import jsonc as json
except ImportError:
    import json


class QReceptionTopicWidget(QWidget):
    def __init__(self, topic_config: dict):
        super().__init__()

        self.topic_config = topic_config
        self.topic_name = topic_config['name']

        self.init_ui()

    def init_ui(self):
        # メインレイアウト
        self.main_layout = QHBoxLayout()

        # 左側のチェックボックスとトピック名をまとめるレイアウト
        left_layout = QHBoxLayout()

        # チェックボックス
        self.checkbox = QCheckBox()
        self.checkbox.setStyleSheet("QCheckBox::indicator { width: 25px; height: 25px; }")
        self.checkbox.setChecked(True)
        left_layout.addWidget(self.checkbox)

        # トピック名
        self.topic_name_label = QLabel(self.topic_name)
        left_layout.addWidget(self.topic_name_label)

        # 左側レイアウトをメインレイアウトに追加
        self.main_layout.addLayout(left_layout)

        # 最後のメッセージ
        self.last_message_label = QLabel()

        self.last_message_label.setStyleSheet("QLabel { background-color: #ffffff; border: 1px solid #aaaaaa; }")
        self.last_message_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.last_message_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.main_layout.addWidget(self.last_message_label)

        self.setLayout(self.main_layout)

        
class QReceptionWidget(QWidget):
    def __init__(self, config_path):
        super().__init__()

        with open(config_path, 'r') as f:
            self.transmission_topics = json.load(f)["reception_messages"]

        self.init_ui()

    def init_ui(self):
        self.main_layout = QVBoxLayout()

        self.topic_widgets: dict[str, QReceptionTopicWidget] = {}
        for topic_config in self.transmission_topics:
            topic_widget = QReceptionTopicWidget(topic_config)
            self.topic_widgets[topic_widget.topic_name] = topic_widget
            self.main_layout.addWidget(topic_widget)
        self.setLayout(self.main_layout)

    def is_active_topic(self, topic_name: str) -> bool:
        '''
        アクティブなトピック名のリストを取得する
        '''
        if topic_name not in self.topic_widgets:
            return False
        return self.topic_widgets[topic_name].checkbox.isChecked()


if __name__ == "__main__":
    app = QApplication([])
    widget = QReceptionWidget()
    widget.show()
    app.exec()

