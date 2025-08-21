"""
Control module for high-level computation and processing.

This package contains the modules responsible for computationally intensive tasks
such as sensor fusion and kinematics calculations, separate from the GUI thread.
"""

# Make the ML processor available for import
from .ml_processor import MLHighLevelProcessor 