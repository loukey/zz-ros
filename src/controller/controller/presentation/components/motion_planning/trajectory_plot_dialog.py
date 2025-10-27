"""
轨迹曲线显示对话框
"""
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QTabWidget, QWidget, QLabel)
from PyQt5.QtCore import Qt
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np


class TrajectoryPlotDialog(QDialog):
    """
    轨迹曲线显示对话框
    
    显示位置、速度、加速度曲线
    """
    
    def __init__(self, parent=None, trajectory_data=None, context=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            trajectory_data: 轨迹数据字典 {
                "time": [...],
                "positions": [[q1,...,q6], ...],
                "velocities": [[qd1,...,qd6], ...],
                "accelerations": [[qdd1,...,qdd6], ...]
            }
            context: 上下文信息 {"type": "node|plan", ...}
        """
        super().__init__(parent)
        self.trajectory_data = trajectory_data or {}
        self.context = context or {}
        
        self._setup_ui()
        self._plot_trajectories()
    
    def _setup_ui(self):
        """设置UI"""
        # 设置窗口标题
        plot_type = "Node" if self.context.get("type") == "node" else "Plan"
        self.setWindowTitle(f"Trajectory Plot - {plot_type}")
        self.resize(1200, 800)
        
        layout = QVBoxLayout(self)
        
        # 信息标签
        info_layout = QHBoxLayout()
        time_points = len(self.trajectory_data.get("time", []))
        duration = self.trajectory_data.get("time", [0])[-1] if time_points > 0 else 0
        info_label = QLabel(f"Points: {time_points}  Duration: {duration:.2f}s")
        info_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        info_layout.addWidget(info_label)
        info_layout.addStretch()
        layout.addLayout(info_layout)
        
        # 创建Tab Widget
        self.tab_widget = QTabWidget()
        
        # 位置曲线Tab
        self.position_tab = self._create_plot_tab("Position (rad)")
        self.tab_widget.addTab(self.position_tab, "Position")
        
        # 速度曲线Tab
        self.velocity_tab = self._create_plot_tab("Velocity (rad/s)")
        self.tab_widget.addTab(self.velocity_tab, "Velocity")
        
        # 加速度曲线Tab
        self.acceleration_tab = self._create_plot_tab("Acceleration (rad/s²)")
        self.tab_widget.addTab(self.acceleration_tab, "Acceleration")
        
        layout.addWidget(self.tab_widget)
        
        # 关闭按钮
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        close_btn.setFixedWidth(100)
        
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)
    
    def _create_plot_tab(self, ylabel: str):
        """
        创建一个绘图Tab
        
        Args:
            ylabel: Y轴标签
            
        Returns:
            QWidget
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 创建matplotlib图形
        figure = Figure(figsize=(12, 8))
        canvas = FigureCanvas(figure)
        layout.addWidget(canvas)
        
        # 保存引用以便后续绘图
        widget.figure = figure
        widget.canvas = canvas
        widget.ylabel = ylabel
        
        return widget
    
    def _plot_trajectories(self):
        """绘制轨迹曲线"""
        time = self.trajectory_data.get("time", [])
        positions = self.trajectory_data.get("positions", [])
        velocities = self.trajectory_data.get("velocities", [])
        accelerations = self.trajectory_data.get("accelerations", [])
        
        if not time or not positions:
            return
        
        # 转换为numpy数组
        time = np.array(time)
        positions = np.array(positions)
        velocities = np.array(velocities) if velocities else np.zeros_like(positions)
        accelerations = np.array(accelerations) if accelerations else np.zeros_like(positions)
        
        # 绘制位置曲线
        self._plot_6_joints(self.position_tab, time, positions)
        
        # 绘制速度曲线
        self._plot_6_joints(self.velocity_tab, time, velocities)
        
        # 绘制加速度曲线
        self._plot_6_joints(self.acceleration_tab, time, accelerations)
    
    def _plot_6_joints(self, tab_widget, time, data):
        """
        绘制6个关节的曲线
        
        Args:
            tab_widget: Tab Widget
            time: 时间数组
            data: 数据数组 (N, 6)
        """
        figure = tab_widget.figure
        figure.clear()
        
        # 创建3x2的子图
        joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        for i in range(6):
            ax = figure.add_subplot(3, 2, i + 1)
            
            if len(data) > 0 and data.shape[1] > i:
                ax.plot(time, data[:, i], color=colors[i], linewidth=1.5, label=joint_names[i])
                ax.set_xlabel('Time (s)', fontsize=10)
                ax.set_ylabel(tab_widget.ylabel, fontsize=10)
                ax.set_title(joint_names[i], fontsize=12, fontweight='bold')
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right')
            else:
                ax.text(0.5, 0.5, 'No Data', 
                       horizontalalignment='center',
                       verticalalignment='center',
                       transform=ax.transAxes,
                       fontsize=14)
                ax.set_title(joint_names[i], fontsize=12, fontweight='bold')
        
        figure.tight_layout()
        tab_widget.canvas.draw()

