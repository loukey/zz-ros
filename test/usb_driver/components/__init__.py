"""
UI组件模块
"""
from .control_components import ControlButtonsFrame, AngleControlFrame
from .display_components import DataDisplayFrame
from .kinematic_components import InverseKinematicFrame, CurvePlotFrame, EndPositionFrame
from .serial_components import PortSelectionFrame, SerialConfigFrame
from .effector_components import EffectorSettings
from .contour_components import ContourSettings

__all__ = [
    "ControlButtonsFrame",
    "AngleControlFrame",
    "DataDisplayFrame",
    "InverseKinematicFrame",
    "CurvePlotFrame",
    "EndPositionFrame",
    "PortSelectionFrame",
    "SerialConfigFrame",
    "EffectorSettings",
    "ContourSettings"
]

