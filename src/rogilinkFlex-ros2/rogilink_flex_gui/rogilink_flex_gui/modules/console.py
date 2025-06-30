from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


class QConsole(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        #self.setStyleSheet("QTextEdit { background-color: #000000; color: #efefef; }")
        
    def append(self, text : str):
        '''テキストを追加'''
        super().append(text)
        self.moveCursor(QTextCursor.MoveOperation.End)
