from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt

class LightPalette(QPalette):
    def __init__(self):
        super().__init__()
        self.setColor(QPalette.Window, QColor(240, 240, 240))
        self.setColor(QPalette.WindowText, QColor(0, 0, 0))
        self.setColor(QPalette.Base, QColor(255, 255, 255))
        self.setColor(QPalette.AlternateBase, QColor(245, 245, 245))
        self.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
        self.setColor(QPalette.ToolTipText, QColor(0, 0, 0))
        self.setColor(QPalette.Text, QColor(0, 0, 0))
        self.setColor(QPalette.Button, QColor(240, 240, 240))
        self.setColor(QPalette.ButtonText, QColor(0, 0, 0))
        self.setColor(QPalette.BrightText, QColor(255, 0, 0))
        self.setColor(QPalette.Link, QColor(0, 0, 255))
        self.setColor(QPalette.Highlight, QColor(0, 120, 215))
        self.setColor(QPalette.HighlightedText, QColor(255, 255, 255)) 