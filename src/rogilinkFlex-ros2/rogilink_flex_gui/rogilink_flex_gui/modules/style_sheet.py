QSS_STYLE_SHEET = """
*{
    font-family: Menlo;
}

/* 全体の背景色とテキスト色 */
QWidget {
    background-color: #1c252f;
    color: #EFEFEF;
}


QTabWidget > QWidget {
    border: none;
}
QTabWidget::pane {
  border: none;
}

QCheckBox,
QSpinBox,
QComboBox {
    border: 2px solid #2A3E50;
    padding: 5px 15px;
    border-radius: 4px;
}

QPushButton {
    background-color: #356087;
    border: 2px solid #2A3E50;
    padding: 5px 15px;
    border-radius: 4px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #4A6A8E;
}
QPushButton:pressed {
    background-color: #224466;
}

QLineEdit,
QTextEdit {
    background-color: #12181f;
    border: 2px solid #276299;
    border-radius: 4px;
    padding: 2px 5px;
}


QSplitter::handle {
    background-color: #060b11;
    border: 1px solid #111d29;
}

QSplitter::handle:vertical {
    width: 6px;
}

QSplitter::handle:hover {
    background-color: #5B7BA0;  /* ホバー時はもう少し明るいブルー */
}


QCheckBox {
    color: #ffffff;
    background-color: transparent;
}


QTabBar{
  text-transform: uppercase;
  font-weight: bold;
}

QTabBar::tab {
  color: #ffffff;
  border: 0px;
}

QTabBar::tab:bottom,
QTabBar::tab:top{
    padding: 0 16px;
    height: 28px;
}
QTabBar::tab:left,
QTabBar::tab:right{
    padding: 16px 0;
    width: 28px;
}

QTabBar::tab:top:selected,
QTabBar::tab:top:hover {
    color: #1859cb;
    border-bottom: 2px solid #1859cb;
}

QTabBar::tab:bottom:selected,
QTabBar::tab:bottom:hover {
    color: #1859cb;
    border-top: 2px solid #1859cb;
}

QTabBar::tab:right:selected,
QTabBar::tab:right:hover {
    color: #1859cb;
    border-left: 2px solid #1859cb;
}

QTabBar::tab:left:selected,
QTabBar::tab:left:hover {
    color: #1859cb;
    border-right: 2px solid #1859cb;
}

QTabBar QToolButton:hover,
QTabBar QToolButton {
    border: 0px;
    background-color: #232629;
    background: #232629;
}
"""