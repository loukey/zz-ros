"""
全局主题定义 - 颜色、布局常量、QSS样式表
"""
import platform

# === 跨平台字体 ===
_is_windows = platform.system() == 'Windows'

# UI 字体（中文支持）
FONT_FAMILY = 'Microsoft YaHei' if _is_windows else 'Noto Sans CJK SC'
# QSS 回退链
FONT_FAMILY_CSS = (
    '"Microsoft YaHei", "SimHei", sans-serif' if _is_windows
    else '"Noto Sans CJK SC", "WenQuanYi Micro Hei", "Source Han Sans SC", sans-serif'
)
# 等宽字体
MONO_FAMILY = 'Consolas' if _is_windows else 'Noto Sans Mono'
MONO_FAMILY_CSS = (
    'Consolas, "Courier New", monospace' if _is_windows
    else '"Noto Sans Mono", "DejaVu Sans Mono", "Liberation Mono", monospace'
)

# === 颜色常量 ===
PRIMARY = "#1976D2"
PRIMARY_DARK = "#1565C0"
PRIMARY_LIGHT = "#42A5F5"
DANGER = "#E53935"
DANGER_DARK = "#C62828"
SUCCESS = "#43A047"
SUCCESS_DARK = "#2E7D32"

BG_WIDGET = "#FAFAFA"
BG_INPUT = "#FFFFFF"
BG_GROUP = "#FFFFFF"
BORDER = "#BDBDBD"
BORDER_LIGHT = "#E0E0E0"
BORDER_FOCUS = PRIMARY
TEXT_PRIMARY = "#212121"
TEXT_SECONDARY = "#757575"
TEXT_DISABLED = "#9E9E9E"

# === 消息颜色 ===
MSG_SEND = "#1565C0"      # 发送=蓝色
MSG_RECEIVE = "#2E7D32"   # 接收=绿色
MSG_ERROR = "#C62828"      # 错误=红色
MSG_SYSTEM = "#6A1B9A"     # 系统=紫色

MSG_COLOR_MAP = {
    "发送": MSG_SEND,
    "控制": MSG_SEND,
    "参数": MSG_SEND,
    "轨迹": MSG_SEND,
    "调试": MSG_SEND,
    "摄像头": MSG_SEND,
    "接收": MSG_RECEIVE,
    "信息": MSG_RECEIVE,
    "错误": MSG_ERROR,
    "系统": MSG_SYSTEM,
}

# === 布局常量 ===
MARGIN_TAB = 8
SPACING_SECTION = 6
MARGIN_GROUP = (8, 14, 8, 8)  # top=14 给 GroupBox title 留空
SPACING_GROUP = 6
MARGIN_MAIN = (4, 4, 4, 4)


def get_stylesheet() -> str:
    return f"""
    /* === 全局字体 === */
    QWidget {{
        font-family: {FONT_FAMILY_CSS};
    }}

    /* === QGroupBox === */
    QGroupBox {{
        border: 1px solid {BORDER};
        border-radius: 5px;
        margin-top: 10px;
        padding: 18px 6px 6px 6px;
        background-color: {BG_GROUP};
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 10px;
        top: 2px;
        padding: 0 6px;
        color: {PRIMARY};
        font-weight: bold;
    }}

    /* === QPushButton === */
    QPushButton {{
        border: 1px solid {BORDER};
        border-radius: 4px;
        padding: 5px 14px;
        background-color: {BG_INPUT};
        color: {TEXT_PRIMARY};
    }}
    QPushButton:hover {{
        background-color: #E3F2FD;
        border-color: {PRIMARY_LIGHT};
    }}
    QPushButton:pressed {{
        background-color: #BBDEFB;
        border-color: {PRIMARY};
    }}
    QPushButton:disabled {{
        background-color: #F5F5F5;
        color: {TEXT_DISABLED};
        border-color: {BORDER_LIGHT};
    }}
    QPushButton:checked {{
        background-color: {PRIMARY};
        color: white;
        border-color: {PRIMARY_DARK};
    }}

    /* === QLineEdit / QSpinBox / QDoubleSpinBox / QComboBox === */
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
        border: 1px solid {BORDER};
        border-radius: 3px;
        padding: 3px 6px;
        background-color: {BG_INPUT};
        color: {TEXT_PRIMARY};
    }}
    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
        border-color: {PRIMARY};
    }}
    QComboBox::drop-down {{
        border: none;
        width: 20px;
    }}

    /* === QTabWidget === */
    QTabWidget::pane {{
        border: 1px solid {BORDER};
        border-top: 2px solid {PRIMARY};
        background-color: {BG_WIDGET};
    }}
    QTabBar::tab {{
        padding: 6px 16px;
        margin-right: 2px;
        border: 1px solid {BORDER_LIGHT};
        border-bottom: none;
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        background-color: #ECEFF1;
        color: {TEXT_SECONDARY};
    }}
    QTabBar::tab:selected {{
        background-color: {BG_INPUT};
        color: {PRIMARY};
        font-weight: bold;
        border-color: {BORDER};
    }}
    QTabBar::tab:hover:!selected {{
        background-color: #E3F2FD;
        color: {PRIMARY_LIGHT};
    }}

    /* === QTextEdit === */
    QTextEdit {{
        border: 1px solid {BORDER};
        border-radius: 3px;
        background-color: {BG_INPUT};
        font-family: {MONO_FAMILY_CSS};
    }}

    /* === QSplitter === */
    QSplitter::handle {{
        background-color: {BORDER_LIGHT};
        width: 3px;
        height: 3px;
    }}
    QSplitter::handle:hover {{
        background-color: {PRIMARY_LIGHT};
    }}

    /* === QScrollBar (vertical) === */
    QScrollBar:vertical {{
        border: none;
        background: #F5F5F5;
        width: 8px;
        margin: 0;
    }}
    QScrollBar::handle:vertical {{
        background: {BORDER};
        border-radius: 4px;
        min-height: 30px;
    }}
    QScrollBar::handle:vertical:hover {{
        background: {TEXT_SECONDARY};
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
    }}

    /* === QScrollBar (horizontal) === */
    QScrollBar:horizontal {{
        border: none;
        background: #F5F5F5;
        height: 8px;
        margin: 0;
    }}
    QScrollBar::handle:horizontal {{
        background: {BORDER};
        border-radius: 4px;
        min-width: 30px;
    }}
    QScrollBar::handle:horizontal:hover {{
        background: {TEXT_SECONDARY};
    }}
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0;
    }}

    /* === QMenuBar === */
    QMenuBar {{
        background-color: {BG_INPUT};
        border-bottom: 1px solid {BORDER_LIGHT};
    }}
    QMenuBar::item:selected {{
        background-color: #E3F2FD;
        color: {PRIMARY};
    }}
    QMenu {{
        background-color: {BG_INPUT};
        border: 1px solid {BORDER};
    }}
    QMenu::item:selected {{
        background-color: {PRIMARY};
        color: white;
    }}
    """
