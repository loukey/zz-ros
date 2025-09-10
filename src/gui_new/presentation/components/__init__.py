"""
UI组件模块（聚合导出）
"""
# Main tab
from .main.port_selection import PortSelectionFrame
from .main.control_components import ControlButtonsFrame, AngleControlFrame

# Effector (main tab section)
from .effector.effector_components import EffectorFrame

# Motion Planning tab
from .motion_planning.motion_planning_components import (
    MotionPlanningFrame,
    MotionPlanningTable,
    MotionPointDialog,
)

# Camera tab
from .camera.camera_components import CameraFrame, CameraDisplayWidget

# Dynamics tab
from .dynamics.dynamics_components import DynamicsFrame

# Right display panel
from .display.data_display import DataDisplayFrame

# Settings dialogs
from .settings.contour_components import ContourSettings
from .settings.serial_components import SerialConfigFrame

# Status area
from .status.status_component import StatusDisplayComponent, StatusSeparator

__all__ = [
    # Main
    "PortSelectionFrame",
    "ControlButtonsFrame",
    "AngleControlFrame",
    # Effector
    "EffectorFrame",
    # Motion planning
    "MotionPlanningFrame",
    "MotionPlanningTable",
    "MotionPointDialog",
    # Camera
    "CameraFrame",
    "CameraDisplayWidget",
    # Dynamics
    "DynamicsFrame",
    # Display
    "DataDisplayFrame",
    # Settings
    "ContourSettings",
    "SerialConfigFrame",
    # Status
    "StatusDisplayComponent",
    "StatusSeparator",
]
