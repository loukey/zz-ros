#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OBB (Oriented Bounding Box) 标注工具
支持手动标注旋转边界框，生成YOLO OBB格式训练数据
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
import cv2
import numpy as np
from PIL import Image, ImageTk
import os
import json
import math
from pathlib import Path
from typing import List, Tuple, Optional, Dict


class OBBAnnotation:
    """OBB标注数据类"""
    
    def __init__(self, class_id: int, points: List[Tuple[float, float]], class_name: str = "part"):
        self.class_id = class_id
        self.points = points  # 4个点的坐标 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        self.class_name = class_name
    
    def to_yolo_format(self, img_width: int, img_height: int) -> str:
        """转换为YOLO OBB格式：class_id x1 y1 x2 y2 x3 y3 x4 y4 (归一化坐标)"""
        normalized_points = []
        for x, y in self.points:
            norm_x = x / img_width
            norm_y = y / img_height
            normalized_points.extend([norm_x, norm_y])
        
        coords_str = ' '.join([f'{coord:.6f}' for coord in normalized_points])
        return f"{self.class_id} {coords_str}"
    
    @classmethod
    def from_yolo_format(cls, line: str, img_width: int, img_height: int, class_name: str = "part"):
        """从YOLO OBB格式创建标注"""
        parts = line.strip().split()
        class_id = int(parts[0])
        coords = [float(x) for x in parts[1:]]
        
        points = []
        for i in range(0, len(coords), 2):
            x = coords[i] * img_width
            y = coords[i + 1] * img_height
            points.append((x, y))
        
        return cls(class_id, points, class_name)


class OBBAnnotationTool:
    """OBB标注工具主类"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("OBB标注工具 - Oriented Bounding Box Annotation Tool")
        self.root.geometry("1400x900")
        
        # 数据存储
        self.current_image = None
        self.current_image_path = None
        self.image_list = []
        self.current_image_index = 0
        self.annotations: List[OBBAnnotation] = []
        self.classes = {"part": 0}  # 默认只有一个类别
        self.current_class = "part"
        
        # 绘制状态
        self.drawing = False
        self.current_box_points = []
        self.selected_annotation = None
        self.drag_point_index = None
        
        # 图像显示相关
        self.display_scale = 1.0
        self.canvas_width = 800
        self.canvas_height = 600
        
        # 状态栏变量（必须在create_widgets之前初始化）
        self.status_var = tk.StringVar()
        self.status_var.set("就绪 - 请加载图像开始标注")
        
        # 创建界面
        self.create_widgets()
        self.bind_events()
    
    def create_widgets(self):
        """创建界面组件"""
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左侧工具面板
        left_frame = ttk.Frame(main_frame, width=250)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_frame.pack_propagate(False)
        
        # 右侧图像显示区域
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 创建左侧工具面板
        self.create_tool_panel(left_frame)
        
        # 创建图像显示区域
        self.create_image_panel(right_frame)
        
        # 状态栏
        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(status_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=5, pady=2)
    
    def create_tool_panel(self, parent):
        """创建左侧工具面板"""
        # 文件操作区域
        file_frame = ttk.LabelFrame(parent, text="文件操作")
        file_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(file_frame, text="加载图像目录", command=self.load_image_directory).pack(fill=tk.X, pady=2)
        ttk.Button(file_frame, text="加载单张图像", command=self.load_single_image).pack(fill=tk.X, pady=2)
        ttk.Button(file_frame, text="保存标注", command=self.save_annotations).pack(fill=tk.X, pady=2)
        ttk.Button(file_frame, text="加载标注", command=self.load_annotations).pack(fill=tk.X, pady=2)
        
        # 图像导航
        nav_frame = ttk.LabelFrame(parent, text="图像导航")
        nav_frame.pack(fill=tk.X, pady=(0, 10))
        
        nav_buttons_frame = ttk.Frame(nav_frame)
        nav_buttons_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(nav_buttons_frame, text="上一张", command=self.prev_image).pack(side=tk.LEFT, padx=2)
        ttk.Button(nav_buttons_frame, text="下一张", command=self.next_image).pack(side=tk.RIGHT, padx=2)
        
        self.image_info_var = tk.StringVar()
        self.image_info_var.set("0 / 0")
        ttk.Label(nav_frame, textvariable=self.image_info_var).pack(pady=2)
        
        # 图像列表
        list_frame = ttk.Frame(nav_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=2)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.image_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, height=6)
        self.image_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.image_listbox.yview)
        
        self.image_listbox.bind('<<ListboxSelect>>', self.on_image_select)
        
        # 类别管理
        class_frame = ttk.LabelFrame(parent, text="类别管理")
        class_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.class_var = tk.StringVar(value="part")
        ttk.Label(class_frame, text="当前类别:").pack(anchor=tk.W)
        self.class_combo = ttk.Combobox(class_frame, textvariable=self.class_var, values=list(self.classes.keys()))
        self.class_combo.pack(fill=tk.X, pady=2)
        self.class_combo.bind('<<ComboboxSelected>>', self.on_class_change)
        
        class_buttons_frame = ttk.Frame(class_frame)
        class_buttons_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(class_buttons_frame, text="添加类别", command=self.add_class).pack(side=tk.LEFT, padx=2)
        ttk.Button(class_buttons_frame, text="删除类别", command=self.delete_class).pack(side=tk.RIGHT, padx=2)
        
        # 标注操作
        annotation_frame = ttk.LabelFrame(parent, text="标注操作")
        annotation_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(annotation_frame, text="清除所有标注", command=self.clear_annotations).pack(fill=tk.X, pady=2)
        ttk.Button(annotation_frame, text="删除选中标注", command=self.delete_selected_annotation).pack(fill=tk.X, pady=2)
        
        # 标注列表
        annotations_frame = ttk.LabelFrame(parent, text="当前标注")
        annotations_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        annotations_list_frame = ttk.Frame(annotations_frame)
        annotations_list_frame.pack(fill=tk.BOTH, expand=True, pady=2)
        
        ann_scrollbar = ttk.Scrollbar(annotations_list_frame)
        ann_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.annotations_listbox = tk.Listbox(annotations_list_frame, yscrollcommand=ann_scrollbar.set)
        self.annotations_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ann_scrollbar.config(command=self.annotations_listbox.yview)
        
        self.annotations_listbox.bind('<<ListboxSelect>>', self.on_annotation_select)
        
        # 帮助信息
        help_frame = ttk.LabelFrame(parent, text="操作说明")
        help_frame.pack(fill=tk.X, pady=(0, 5))
        
        help_text = """操作方法:
• 鼠标左键：点击4个点绘制OBB
• 鼠标右键：取消当前绘制
• 拖拽：移动已标注的点
• Delete键：删除选中标注
• 方向键：切换图像
• Ctrl+S：保存标注"""
        
        ttk.Label(help_frame, text=help_text, justify=tk.LEFT, font=("Arial", 8)).pack(anchor=tk.W, padx=5, pady=5)
    
    def create_image_panel(self, parent):
        """创建图像显示面板"""
        # 工具栏
        toolbar = ttk.Frame(parent)
        toolbar.pack(side=tk.TOP, fill=tk.X, pady=(0, 5))
        
        ttk.Button(toolbar, text="适应窗口", command=self.fit_to_window).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="原始大小", command=self.original_size).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="放大", command=self.zoom_in).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="缩小", command=self.zoom_out).pack(side=tk.LEFT, padx=2)
        
        self.zoom_var = tk.StringVar()
        self.zoom_var.set("100%")
        ttk.Label(toolbar, textvariable=self.zoom_var).pack(side=tk.RIGHT, padx=5)
        
        # 图像显示画布
        canvas_frame = ttk.Frame(parent)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建滚动画布
        self.canvas = tk.Canvas(canvas_frame, bg='gray', cursor='crosshair')
        
        v_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        
        self.canvas.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)
        
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    
    def bind_events(self):
        """绑定事件"""
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Motion>", self.on_canvas_motion)
        
        self.root.bind("<Key>", self.on_key_press)
        self.root.bind("<Control-s>", lambda e: self.save_annotations())
        self.root.bind("<Delete>", lambda e: self.delete_selected_annotation())
        self.root.bind("<Left>", lambda e: self.prev_image())
        self.root.bind("<Right>", lambda e: self.next_image())
        
        self.root.focus_set()  # 确保窗口能接收键盘事件
    
    def load_image_directory(self):
        """加载图像目录"""
        directory = filedialog.askdirectory(title="选择图像目录")
        if directory:
            self.image_list = []
            for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
                self.image_list.extend(Path(directory).glob(ext))
                self.image_list.extend(Path(directory).glob(ext.upper()))
            
            self.image_list = [str(p) for p in self.image_list]
            self.image_list.sort()
            
            if self.image_list:
                self.current_image_index = 0
                self.update_image_list()
                self.load_current_image()
                self.status_var.set(f"已加载 {len(self.image_list)} 张图像")
            else:
                messagebox.showwarning("警告", "在选定目录中未找到图像文件")
    
    def load_single_image(self):
        """加载单张图像"""
        file_path = filedialog.askopenfilename(
            title="选择图像文件",
            filetypes=[("图像文件", "*.jpg *.jpeg *.png *.bmp"), ("所有文件", "*.*")]
        )
        if file_path:
            self.image_list = [file_path]
            self.current_image_index = 0
            self.update_image_list()
            self.load_current_image()
            self.status_var.set("已加载单张图像")
    
    def update_image_list(self):
        """更新图像列表显示"""
        self.image_listbox.delete(0, tk.END)
        for img_path in self.image_list:
            self.image_listbox.insert(tk.END, Path(img_path).name)
        
        if self.image_list:
            self.image_listbox.selection_set(self.current_image_index)
            self.image_info_var.set(f"{self.current_image_index + 1} / {len(self.image_list)}")
        else:
            self.image_info_var.set("0 / 0")
    
    def load_current_image(self):
        """加载当前图像"""
        if not self.image_list or self.current_image_index >= len(self.image_list):
            return
        
        # 保存当前标注
        if self.current_image_path:
            self.auto_save_annotations()
        
        image_path = self.image_list[self.current_image_index]
        self.current_image_path = image_path
        
        # 加载图像
        self.current_image = cv2.imread(image_path)
        self.current_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB)
        
        # 自动加载对应的标注文件
        self.auto_load_annotations()
        
        # 显示图像
        self.display_image()
        self.update_annotations_list()
        
        self.status_var.set(f"已加载: {Path(image_path).name}")
    
    def display_image(self):
        """显示图像到画布"""
        if self.current_image is None:
            return
        
        # 计算显示尺寸
        img_height, img_width = self.current_image.shape[:2]
        
        # 创建PIL图像
        pil_image = Image.fromarray(self.current_image)
        
        # 根据缩放比例调整图像
        display_width = int(img_width * self.display_scale)
        display_height = int(img_height * self.display_scale)
        
        if self.display_scale != 1.0:
            pil_image = pil_image.resize((display_width, display_height), Image.Resampling.LANCZOS)
        
        # 转换为Tkinter可用的格式
        self.photo = ImageTk.PhotoImage(pil_image)
        
        # 清除画布并显示图像
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
        
        # 设置滚动区域
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        
        # 绘制标注
        self.draw_annotations()
        
        # 更新缩放显示
        self.zoom_var.set(f"{int(self.display_scale * 100)}%")
    
    def draw_annotations(self):
        """绘制所有标注"""
        if self.current_image is None:
            return
        
        # 清除所有标注相关的绘制元素（使用更高效的方式）
        self.canvas.delete("annotation")  # 删除所有带annotation标签的元素
        
        colors = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'cyan', 'magenta']
        
        for i, annotation in enumerate(self.annotations):
            color = colors[i % len(colors)]
            
            # 绘制多边形
            scaled_points = [(x * self.display_scale, y * self.display_scale) for x, y in annotation.points]
            
            if len(scaled_points) >= 4:
                # 绘制边框
                self.canvas.create_polygon(scaled_points, outline=color, fill='', width=2, tags=("annotation", f"annotation_{i}"))
                
                # 绘制顶点
                for j, (x, y) in enumerate(scaled_points):
                    self.canvas.create_oval(x-3, y-3, x+3, y+3, fill=color, tags=("annotation", f"point_{i}_{j}"))
                
                # 添加类别标签
                center_x = sum(x for x, y in scaled_points) / len(scaled_points)
                center_y = sum(y for x, y in scaled_points) / len(scaled_points)
                self.canvas.create_text(center_x, center_y, text=annotation.class_name, 
                                      fill=color, font=("Arial", 10, "bold"), tags=("annotation", f"label_{i}"))
        
        # 绘制正在绘制的框
        if self.current_box_points:
            temp_color = 'white'
            scaled_temp_points = [(x * self.display_scale, y * self.display_scale) for x, y in self.current_box_points]
            
            for x, y in scaled_temp_points:
                self.canvas.create_oval(x-3, y-3, x+3, y+3, fill=temp_color, tags=("annotation", "temp_point"))
            
            if len(scaled_temp_points) >= 2:
                for i in range(len(scaled_temp_points) - 1):
                    x1, y1 = scaled_temp_points[i]
                    x2, y2 = scaled_temp_points[i + 1]
                    self.canvas.create_line(x1, y1, x2, y2, fill=temp_color, width=2, tags=("annotation", "temp_line"))
                
                # 如果有4个点，连接最后一个点到第一个点
                if len(scaled_temp_points) == 4:
                    x1, y1 = scaled_temp_points[-1]
                    x2, y2 = scaled_temp_points[0]
                    self.canvas.create_line(x1, y1, x2, y2, fill=temp_color, width=2, tags=("annotation", "temp_line"))
    
    def on_canvas_click(self, event):
        """画布点击事件"""
        if self.current_image is None:
            return
        
        # 转换坐标
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        
        # 转换为图像坐标
        img_x = canvas_x / self.display_scale
        img_y = canvas_y / self.display_scale
        
        # 检查是否点击了现有的点
        clicked_point = self.find_clicked_point(canvas_x, canvas_y)
        if clicked_point:
            self.selected_annotation = clicked_point[0]
            self.drag_point_index = clicked_point[1]
            self.drawing = False
            return
        
        # 检查是否点击了现有的标注
        clicked_annotation = self.find_clicked_annotation(canvas_x, canvas_y)
        if clicked_annotation is not None:
            self.selected_annotation = clicked_annotation
            self.update_annotations_list()
            return
        
        # 绘制新的OBB
        if not self.drawing:
            self.drawing = True
            self.current_box_points = []
            self.selected_annotation = None
        
        # 添加点
        self.current_box_points.append((img_x, img_y))
        
        # 如果已经绘制了4个点，完成标注
        if len(self.current_box_points) == 4:
            self.finish_annotation()
        
        self.draw_annotations()
        self.update_annotations_list()
    
    def on_canvas_right_click(self, event):
        """右键点击事件 - 取消当前绘制"""
        self.cancel_current_drawing()
    
    def on_canvas_drag(self, event):
        """画布拖拽事件"""
        if self.drag_point_index is not None and self.selected_annotation is not None:
            canvas_x = self.canvas.canvasx(event.x)
            canvas_y = self.canvas.canvasy(event.y)
            
            img_x = canvas_x / self.display_scale
            img_y = canvas_y / self.display_scale
            
            # 更新点的位置
            self.annotations[self.selected_annotation].points[self.drag_point_index] = (img_x, img_y)
            self.draw_annotations()
    
    def on_canvas_release(self, event):
        """鼠标释放事件"""
        self.drag_point_index = None
    
    def on_canvas_motion(self, event):
        """鼠标移动事件"""
        if self.current_image is not None:
            canvas_x = self.canvas.canvasx(event.x)
            canvas_y = self.canvas.canvasy(event.y)
            
            img_x = int(canvas_x / self.display_scale)
            img_y = int(canvas_y / self.display_scale)
            
            img_height, img_width = self.current_image.shape[:2]
            if 0 <= img_x < img_width and 0 <= img_y < img_height:
                self.status_var.set(f"坐标: ({img_x}, {img_y})")
    
    def find_clicked_point(self, canvas_x, canvas_y, threshold=10):
        """查找点击的点"""
        for i, annotation in enumerate(self.annotations):
            for j, (x, y) in enumerate(annotation.points):
                scaled_x = x * self.display_scale
                scaled_y = y * self.display_scale
                
                if abs(scaled_x - canvas_x) <= threshold and abs(scaled_y - canvas_y) <= threshold:
                    return (i, j)
        return None
    
    def find_clicked_annotation(self, canvas_x, canvas_y):
        """查找点击的标注"""
        img_x = canvas_x / self.display_scale
        img_y = canvas_y / self.display_scale
        
        for i, annotation in enumerate(self.annotations):
            if len(annotation.points) >= 4:
                # 使用OpenCV的pointPolygonTest
                contour = np.array(annotation.points, dtype=np.float32).reshape((-1, 1, 2))
                result = cv2.pointPolygonTest(contour, (img_x, img_y), False)
                if result >= 0:
                    return i
        return None
    
    def finish_annotation(self):
        """完成当前标注"""
        if len(self.current_box_points) == 4:
            class_id = self.classes.get(self.current_class, 0)
            annotation = OBBAnnotation(class_id, self.current_box_points.copy(), self.current_class)
            self.annotations.append(annotation)
            
            self.current_box_points = []
            self.drawing = False
            
            self.status_var.set(f"已添加标注，当前共 {len(self.annotations)} 个标注")
    
    def cancel_current_drawing(self):
        """取消当前绘制"""
        self.current_box_points = []
        self.drawing = False
        self.selected_annotation = None
        self.drag_point_index = None
        self.draw_annotations()
        self.status_var.set("已取消当前绘制")
    
    def clear_annotations(self):
        """清除所有标注"""
        if self.annotations and messagebox.askyesno("确认", "是否清除所有标注？"):
            self.annotations = []
            self.selected_annotation = None
            self.draw_annotations()
            self.update_annotations_list()
            self.status_var.set("已清除所有标注")
    
    def delete_selected_annotation(self):
        """删除选中的标注"""
        if self.selected_annotation is not None and 0 <= self.selected_annotation < len(self.annotations):
            del self.annotations[self.selected_annotation]
            self.selected_annotation = None
            self.draw_annotations()
            self.update_annotations_list()
            self.status_var.set(f"已删除标注，当前共 {len(self.annotations)} 个标注")
    
    def update_annotations_list(self):
        """更新标注列表显示"""
        self.annotations_listbox.delete(0, tk.END)
        for i, annotation in enumerate(self.annotations):
            self.annotations_listbox.insert(tk.END, f"{i+1}. {annotation.class_name}")
        
        if self.selected_annotation is not None:
            self.annotations_listbox.selection_set(self.selected_annotation)
    
    def on_annotation_select(self, event):
        """标注列表选择事件"""
        selection = self.annotations_listbox.curselection()
        if selection:
            self.selected_annotation = selection[0]
            self.draw_annotations()
    
    def on_image_select(self, event):
        """图像列表选择事件"""
        selection = self.image_listbox.curselection()
        if selection:
            new_index = selection[0]
            if new_index != self.current_image_index:
                self.current_image_index = new_index
                self.load_current_image()
                self.update_image_list()
    
    def prev_image(self):
        """上一张图像"""
        if self.image_list and self.current_image_index > 0:
            self.current_image_index -= 1
            self.load_current_image()
            self.update_image_list()
    
    def next_image(self):
        """下一张图像"""
        if self.image_list and self.current_image_index < len(self.image_list) - 1:
            self.current_image_index += 1
            self.load_current_image()
            self.update_image_list()
    
    def fit_to_window(self):
        """适应窗口大小"""
        if self.current_image is None:
            return
        
        img_height, img_width = self.current_image.shape[:2]
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        scale_w = canvas_width / img_width
        scale_h = canvas_height / img_height
        self.display_scale = min(scale_w, scale_h, 1.0)
        
        self.display_image()
    
    def original_size(self):
        """原始大小显示"""
        self.display_scale = 1.0
        self.display_image()
    
    def zoom_in(self):
        """放大"""
        self.display_scale = min(self.display_scale * 1.2, 5.0)
        self.display_image()
    
    def zoom_out(self):
        """缩小"""
        self.display_scale = max(self.display_scale / 1.2, 0.1)
        self.display_image()
    
    def on_class_change(self, event):
        """类别改变事件"""
        self.current_class = self.class_var.get()
    
    def add_class(self):
        """添加新类别"""
        new_class = simpledialog.askstring("添加类别", "请输入新类别名称:")
        if new_class and new_class not in self.classes:
            class_id = len(self.classes)
            self.classes[new_class] = class_id
            self.class_combo['values'] = list(self.classes.keys())
            self.class_var.set(new_class)
            self.current_class = new_class
            self.status_var.set(f"已添加类别: {new_class}")
    
    def delete_class(self):
        """删除类别"""
        if len(self.classes) <= 1:
            messagebox.showwarning("警告", "至少需要保留一个类别")
            return
        
        current_class = self.class_var.get()
        if messagebox.askyesno("确认", f"是否删除类别 '{current_class}'？"):
            if current_class in self.classes:
                del self.classes[current_class]
                # 重新分配ID
                self.classes = {name: i for i, name in enumerate(self.classes.keys())}
                self.class_combo['values'] = list(self.classes.keys())
                if self.classes:
                    first_class = list(self.classes.keys())[0]
                    self.class_var.set(first_class)
                    self.current_class = first_class
                self.status_var.set(f"已删除类别: {current_class}")
    
    def save_annotations(self):
        """保存标注"""
        if not self.current_image_path or not self.annotations or self.current_image is None:
            messagebox.showwarning("警告", "没有可保存的标注")
            return
        
        # 生成标注文件路径
        image_path = Path(self.current_image_path)
        label_dir = image_path.parent / "labels"
        label_dir.mkdir(exist_ok=True)
        
        label_path = label_dir / f"{image_path.stem}.txt"
        
        try:
            img_height, img_width = self.current_image.shape[:2]
            
            with open(label_path, 'w') as f:
                for annotation in self.annotations:
                    line = annotation.to_yolo_format(img_width, img_height)
                    f.write(line + '\n')
            
            # 保存类别信息
            classes_path = label_dir / "classes.txt"
            with open(classes_path, 'w') as f:
                for class_name, class_id in sorted(self.classes.items(), key=lambda x: x[1]):
                    f.write(f"{class_id}: {class_name}\n")
            
            self.status_var.set(f"已保存标注到: {label_path}")
            messagebox.showinfo("成功", f"标注已保存到:\n{label_path}")
            
        except Exception as e:
            messagebox.showerror("错误", f"保存标注失败:\n{str(e)}")
    
    def auto_save_annotations(self):
        """自动保存当前标注"""
        if self.current_image_path and self.annotations and self.current_image is not None:
            image_path = Path(self.current_image_path)
            label_dir = image_path.parent / "labels"
            label_dir.mkdir(exist_ok=True)
            
            label_path = label_dir / f"{image_path.stem}.txt"
            
            try:
                img_height, img_width = self.current_image.shape[:2]
                
                with open(label_path, 'w') as f:
                    for annotation in self.annotations:
                        line = annotation.to_yolo_format(img_width, img_height)
                        f.write(line + '\n')
            except Exception:
                pass  # 静默失败
    
    def load_annotations(self):
        """加载标注"""
        if not self.current_image_path:
            messagebox.showwarning("警告", "请先加载图像")
            return
        
        self.auto_load_annotations()
    
    def auto_load_annotations(self):
        """自动加载标注"""
        if not self.current_image_path or self.current_image is None:
            return
        
        image_path = Path(self.current_image_path)
        label_path = image_path.parent / "labels" / f"{image_path.stem}.txt"
        
        self.annotations = []
        
        if label_path.exists():
            try:
                img_height, img_width = self.current_image.shape[:2]
                
                with open(label_path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line:
                            # 获取类别名称
                            class_id = int(line.split()[0])
                            class_name = "part"  # 默认类别
                            for name, cid in self.classes.items():
                                if cid == class_id:
                                    class_name = name
                                    break
                            
                            annotation = OBBAnnotation.from_yolo_format(line, img_width, img_height, class_name)
                            self.annotations.append(annotation)
                
                self.status_var.set(f"已加载 {len(self.annotations)} 个标注")
                
            except Exception as e:
                self.status_var.set(f"加载标注失败: {str(e)}")
        
        # 加载类别信息
        classes_path = image_path.parent / "labels" / "classes.txt"
        if classes_path.exists():
            try:
                temp_classes = {}
                with open(classes_path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if ':' in line:
                            class_id, class_name = line.split(':', 1)
                            temp_classes[class_name.strip()] = int(class_id.strip())
                
                if temp_classes:
                    self.classes = temp_classes
                    self.class_combo['values'] = list(self.classes.keys())
                    if self.classes:
                        first_class = list(self.classes.keys())[0]
                        self.class_var.set(first_class)
                        self.current_class = first_class
            except Exception:
                pass  # 静默失败
    
    def on_key_press(self, event):
        """键盘按键事件"""
        if event.keysym == 'Escape':
            self.cancel_current_drawing()
        elif event.keysym == 'Delete':
            self.delete_selected_annotation()


def main():
    """主函数"""
    root = tk.Tk()
    app = OBBAnnotationTool(root)
    
    # 设置窗口图标和属性
    try:
        root.state('zoomed')  # Windows最大化
    except:
        root.attributes('-zoomed', True)  # Linux最大化
    
    root.mainloop()


if __name__ == "__main__":
    main() 