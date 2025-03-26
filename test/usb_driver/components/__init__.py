"""
UI组件模块
"""
from .control_components import ControlButtonsFrame, AngleControlFrame
from .display_components import DataDisplayFrame
from .kinematic_components import InverseKinematicFrame, CurvePlotFrame, EndPositionFrame
from .serial_components import PortSelectionFrame, SerialConfigFrame

__all__ = [
    "ControlButtonsFrame",
    "AngleControlFrame",
    "DataDisplayFrame",
    "InverseKinematicFrame",
    "CurvePlotFrame",
    "EndPositionFrame",
    "PortSelectionFrame",
    "SerialConfigFrame"
]

