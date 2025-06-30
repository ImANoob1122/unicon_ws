import os
import typing

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

try:
    import jsonc as json
except ImportError:
    import json


class QTypedLineEdit(QLineEdit):
    def __init__(self, type_: type, default_value: str = "", arg_name: str = ""):
        super().__init__()
        self.type_ = type_
        assert self.type_ in [int, float, str, list[int], list[float]]

        # uiの初期化
        self.init_ui(default_value, arg_name)

        # 1文字入力されたときに呼ばれる
        self.textChanged.connect(self.text_changed_callback)
        self.last_text = default_value

    def init_ui(self, text, arg_name):
        self.setText(text) # default text

        # 背景色を変える
        if self.type_ == int: # 薄い緑
            # placeholderの色を変える
            self.setStyleSheet("QLineEdit { background-color: #8ff0a1;  }")
        elif self.type_ == float: # 薄い水色
            self.setStyleSheet("QLineEdit { background-color: #7cd4f0;  }")
        elif self.type_ in [list[int], list[float]]: # 薄いオレンジ
            self.setStyleSheet("QLineEdit { background-color: #f0b67c;  }")
        elif self.type_ == str: # 薄いグレー
            self.setStyleSheet("QLineEdit { background-color: #d9d9d9; }")

        # プレースホルダーを設定
        if arg_name != "":
            self.setPlaceholderText(arg_name)

    def text_changed_callback(self, text: str):
        if self.check_input(text):
            self.last_text = text
        else:
            self.setText(self.last_text)


    def check_input(self, text: str):
        '''
        入力された文字列が型に合っているかをチェックする
        '''
        if self.type_ == int: # int型の場合
            if text == "" or text == "-":
                return True
            
            if text[0] == "-":
                text = text[1:]
            
            if text.isdigit():
                return True
            
            return False

        elif self.type_ == float: # float型の場合
            if text == "" or text == "-":
                return True

            if text[0] == "-":
                text = text[1:]
            
            if text.count(".") <= 1:
                if text.replace(".", "").isdigit():
                    return True

            
            return False
        
        elif self.type_ == str: # str型の場合
            return True
        
        elif self.type_ == list[int]:
            if text == "":
                return True
            
            for num in text.split(","):
                num = num.strip()
                if num == "" or num == "-":
                    continue
                if num[0] == "-":
                    num = num[1:]
                if not num.isdigit():
                    return False
            return True
        
        elif self.type_ == list[float]:
            if text == "":
                return True
            
            for num in text.split(","):
                num = num.strip()
                if num == "" or num == "-":
                    continue
                if num[0] == "-":
                    num = num[1:]
                if num.count(".") <= 1:
                    if num.replace(".", "").isdigit():
                        continue
                return False
            
            return True
        
    def get_value(self):
        '''
        入力された文字列を型に合わせて返す
        '''
        try:
            if self.type_ == int:
                return int(self.text())
            elif self.type_ == float:
                return float(self.text())
            elif self.type_ == str:
                return self.text()
            elif self.type_ == list[int]:
                return [int(num.strip()) for num in self.text().split(",")]
            elif self.type_ == list[float]:
                return [float(num.strip()) for num in self.text().split(",")]
        except ValueError:
            return None
        


class QTransmissionTopicWidget(QWidget):
    def __init__(self, topic_config: dict, callback: typing.Callable = None):
        super().__init__()
        self.topic_id = topic_config["id"]
        self.topic_name = topic_config["name"]
        self.topic_types = self.str_to_type(topic_config["type"])
        if "args" in topic_config:
            self.arg_names = [arg.strip() for arg in topic_config["args"].split(",")]
        else:
            self.arg_names = [f"arg{i}" for i in range(len(self.topic_types))]
        self.init_ui()
        self.changed_callback()
        self.callback = callback

    def str_to_type(self, type_str: str):
        result = []
        for type_name in type_str.split(","):
            type_name = type_name.strip()
            if type_name in ['int', 'uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64']:
                result.append(int)
            elif type_name in ['float', 'double', 'float32', 'float64']:
                result.append(float)
            elif type_name in ['str', 'char', 'char*']:
                result.append(str)
            elif type_name in ['list[int]', 'list[uint8]', 'list[int8]', 'list[uint16]', 'list[int16]', 'list[uint32]', 'list[int32]', 'list[uint64]', 'list[int64]']:
                result.append(list[int])
            elif type_name in ['list[float]', 'list[double]', 'list[float32]', 'list[float64]']:
                result.append(list[float])
            else:
                result.append(str)
        
        return result

    def init_ui(self):
        self.main_layout = QVBoxLayout()

        # トピックID
        self.topic_name_label = QLabel(f'0x{self.topic_id:02x}: {self.topic_name}')
        self.main_layout.addWidget(self.topic_name_label)

        # トピックの型に合わせた入力欄を作成
        self.input_layout = QHBoxLayout()
        self.input_widgets : list[QTypedLineEdit] = []
        for arg_name, type_ in zip(self.arg_names, self.topic_types):
            input_widget = QTypedLineEdit(type_, default_value="", arg_name=arg_name)
            self.input_layout.addWidget(input_widget)
            self.input_widgets.append(input_widget)
        

        # 送信ボタン
        self.send_button = QPushButton("Send")
        self.input_layout.addWidget(self.send_button)
        self.main_layout.addLayout(self.input_layout)
        self.setLayout(self.main_layout)

        # 送信ボタンが押されたときの処理
        self.send_button.clicked.connect(self.send_button_clicked)

        # 入力欄が変更されたときの処理
        for input_widget in self.input_widgets:
            input_widget.textChanged.connect(self.changed_callback)

    def changed_callback(self):
        '''
        入力欄が変更されたときの処理
        '''
        if None not in self.get_values():
            self.send_button.setEnabled(True)
        else:
            self.send_button.setEnabled(False)

    def get_values(self):
        '''
        入力された値が型に合っているかをチェックする
        '''
        values = []
        for input_widget in self.input_widgets:
            values.append(input_widget.get_value())
        
        return values

    def send_button_clicked(self):
        '''
        送信ボタンが押されたときの処理
        '''
        self.get_values()
        if None not in self.get_values():
            if self.callback is not None:
                value = self.get_values()
                if len(value) == 1:
                    value = value[0]
                self.callback(self.topic_name, value)

    def set_send_button_callback(self, callback: typing.Callable):
        self.callback = callback

class QTransmissionWidget(QWidget):
    def __init__(self, config_path: str):
        super().__init__()

        with open(config_path, 'r') as f:
            self.transmission_topics = json.load(f)["transmission_messages"]

        self.init_ui()

    def init_ui(self):
        self.main_layout = QVBoxLayout()

        self.topic_widgets : dict[str, QTransmissionTopicWidget] = {}
        for topic_config in self.transmission_topics:
            topic_widget = QTransmissionTopicWidget(topic_config)
            self.topic_widgets[topic_widget.topic_name] = topic_widget
            self.main_layout.addWidget(topic_widget)
        self.setLayout(self.main_layout)

    def set_send_message_callback(self, callback: typing.Callable):
        for topic_widget in self.topic_widgets.values():
            topic_widget.set_send_button_callback(callback)
        


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = QTransmissionWidget()
    window.show()
    sys.exit(app.exec())