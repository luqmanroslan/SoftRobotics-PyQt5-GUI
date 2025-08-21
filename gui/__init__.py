"""
Soft Robot Control Interface GUI Package
"""

from .main_window import SoftRobotGUI

__all__ = ['SoftRobotGUI']

import sys
import warnings
from PyQt5.QtWidgets import QApplication

from gui.main_window import SoftRobotGUI
from gui.styles.dark_palette import DarkPalette 