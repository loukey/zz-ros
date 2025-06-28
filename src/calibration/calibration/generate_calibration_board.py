#!/usr/bin/env python3
"""
标定板生成器
支持生成用于相机标定和手眼标定的棋盘格标定板
可输出高质量的PDF和图片格式，适合打印使用
自动支持横向显示以适应大尺寸标定板
"""

import cv2
import numpy as np
import argparse
import os
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4, A3, A2, letter, legal
from reportlab.lib.units import mm, inch
from datetime import datetime


class CalibrationBoardGenerator:
    """标定板生成器类"""
    
    def __init__(self):
        self.supported_paper_sizes = {
            'A4': (210*mm, 297*mm),      # A4: 210 x 297 mm
            'A3': (297*mm, 420*mm),      # A3: 297 x 420 mm  
            'A2': (420*mm, 594*mm),      # A2: 420 x 594 mm
            'Letter': (8.5*inch, 11*inch),    # US Letter: 8.5 x 11 inch
            'Legal': (8.5*inch, 14*inch)      # US Legal: 8.5 x 14 inch
        }
    
    def determine_page_orientation(self, rows, cols, square_size_mm, paper_size):
        """
        确定页面方向（横向或纵向）
        
        Args:
            rows: 棋盘格行数（内角点数）
            cols: 棋盘格列数（内角点数）
            square_size_mm: 方格尺寸（毫米）
            paper_size: 纸张尺寸
            
        Returns:
            tuple: (page_width, page_height, is_landscape)
        """
        if paper_size not in self.supported_paper_sizes:
            raise ValueError(f"不支持的纸张尺寸: {paper_size}")
        
        # 获取纸张原始尺寸
        paper_w, paper_h = self.supported_paper_sizes[paper_size]
        
        # 计算棋盘格尺寸（需要比内角点数多1个方格）
        board_width_mm = (cols + 1) * square_size_mm
        board_height_mm = (rows + 1) * square_size_mm
        
        # 转换为reportlab单位
        board_width = board_width_mm * mm
        board_height = board_height_mm * mm
        
        # 预留10%边距
        margin_ratio = 0.1
        required_width = board_width / (1 - 2 * margin_ratio)
        required_height = board_height / (1 - 2 * margin_ratio)
        
        # 判断是否需要横向显示
        # 纵向：纸张宽度=paper_w, 纸张高度=paper_h
        portrait_fits = (required_width <= paper_w) and (required_height <= paper_h)
        
        # 横向：纸张宽度=paper_h, 纸张高度=paper_w  
        landscape_fits = (required_width <= paper_h) and (required_height <= paper_w)
        
        if portrait_fits:
            # 纵向可以容纳
            return paper_w, paper_h, False, board_width_mm, board_height_mm
        elif landscape_fits:
            # 横向可以容纳
            return paper_h, paper_w, True, board_width_mm, board_height_mm
        else:
            # 都容纳不下，警告但使用横向（更大的空间）
            print(f"警告: 棋盘格尺寸 ({board_width_mm:.1f}x{board_height_mm:.1f}mm) "
                  f"超出纸张尺寸，将使用横向显示")
            return paper_h, paper_w, True, board_width_mm, board_height_mm
    
    def generate_chessboard_image(self, rows, cols, square_size_pixels, margin_pixels=50):
        """
        生成棋盘格图像
        
        Args:
            rows: 棋盘格行数（内角点数）
            cols: 棋盘格列数（内角点数）
            square_size_pixels: 每个方格的像素尺寸
            margin_pixels: 边距像素
            
        Returns:
            numpy.ndarray: 生成的棋盘格图像
        """
        # 计算图像尺寸（需要比内角点数多1个方格）
        board_width = (cols + 1) * square_size_pixels
        board_height = (rows + 1) * square_size_pixels
        
        # 加上边距
        img_width = board_width + 2 * margin_pixels
        img_height = board_height + 2 * margin_pixels
        
        # 创建白色背景图像
        image = np.ones((img_height, img_width), dtype=np.uint8) * 255
        
        # 绘制棋盘格
        for i in range(rows + 1):
            for j in range(cols + 1):
                # 计算方格位置
                y1 = margin_pixels + i * square_size_pixels
                y2 = y1 + square_size_pixels
                x1 = margin_pixels + j * square_size_pixels  
                x2 = x1 + square_size_pixels
                
                # 棋盘格模式：(i+j)为奇数时为黑色
                if (i + j) % 2 == 1:
                    image[y1:y2, x1:x2] = 0  # 黑色方格
        
        return image
    
    def generate_chessboard_pdf(self, rows, cols, square_size_mm, paper_size='A4', filename=None):
        """
        生成棋盘格PDF文件，自动选择最佳页面方向
        
        Args:
            rows: 棋盘格行数（内角点数）
            cols: 棋盘格列数（内角点数）
            square_size_mm: 每个方格的毫米尺寸
            paper_size: 纸张尺寸
            filename: 输出文件名
            
        Returns:
            str: 生成的PDF文件路径
        """
        # 确定页面方向和尺寸
        page_width, page_height, is_landscape, board_width_mm, board_height_mm = \
            self.determine_page_orientation(rows, cols, square_size_mm, paper_size)
        
        # 计算棋盘格尺寸
        board_width = board_width_mm * mm
        board_height = board_height_mm * mm
        
        # 生成文件名
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            orientation = "横向" if is_landscape else "纵向"
            filename = f"chessboard_{rows}x{cols}_{square_size_mm}mm_{paper_size}_{orientation}_{timestamp}.pdf"
        
        # 创建PDF
        c = canvas.Canvas(filename, pagesize=(page_width, page_height))
        
        # 计算居中位置
        start_x = (page_width - board_width) / 2
        start_y = (page_height - board_height) / 2
        
        # 绘制棋盘格
        square_size = square_size_mm * mm
        for i in range(rows + 1):
            for j in range(cols + 1):
                x = start_x + j * square_size
                y = start_y + i * square_size
                
                # 棋盘格模式：(i+j)为奇数时为黑色
                if (i + j) % 2 == 1:
                    c.setFillColor("black")
                    c.rect(x, y, square_size, square_size, fill=1, stroke=0)
        
        # 添加简略信息文本 - 避免覆盖棋盘格
        c.setFillColor("gray")
        c.setFont("Helvetica", 8)
        
        # 计算安全的文本位置（棋盘格下方留出足够空间）
        text_start_y = start_y - 40  # 棋盘格下方40点距离
        
        # 只显示最关键信息
        key_info = f"{rows}x{cols}, {square_size_mm}mm | ROS2: chessboard_size='{cols},{rows}', square_size={square_size_mm/1000:.3f}"
        
        # 如果文本位置太低，就放到页面顶部
        if text_start_y < 50:
            text_y = page_height - 30  # 页面顶部
        else:
            text_y = text_start_y
            
        c.drawString(50, text_y, key_info)
        
        # 保存PDF
        c.save()
        
        # 输出信息
        print(f"✓ PDF已生成: {filename}")
        print(f"  标定板尺寸: {board_width_mm:.1f} x {board_height_mm:.1f} mm")
        print(f"  纸张利用率: {(board_width*board_height)/(page_width*page_height)*100:.1f}%")
        
        return filename
    
    def calculate_optimal_square_size(self, rows, cols, paper_size='A4'):
        """
        根据纸张尺寸计算最佳方格尺寸
        同时考虑纵向和横向两种方向
        
        Args:
            rows: 棋盘格行数
            cols: 棋盘格列数
            paper_size: 纸张尺寸
            
        Returns:
            tuple: (推荐方格尺寸mm, 推荐方向)
        """
        if paper_size not in self.supported_paper_sizes:
            raise ValueError(f"不支持的纸张尺寸: {paper_size}")
        
        paper_w, paper_h = self.supported_paper_sizes[paper_size]
        margin_ratio = 0.1
        
        # 纵向方向计算
        usable_width_portrait = paper_w * (1 - 2 * margin_ratio)
        usable_height_portrait = paper_h * (1 - 2 * margin_ratio)
        max_square_portrait = min(usable_width_portrait / (cols + 1), 
                                 usable_height_portrait / (rows + 1))
        
        # 横向方向计算
        usable_width_landscape = paper_h * (1 - 2 * margin_ratio)
        usable_height_landscape = paper_w * (1 - 2 * margin_ratio)
        max_square_landscape = min(usable_width_landscape / (cols + 1),
                                  usable_height_landscape / (rows + 1))
        
        # 选择更大的方格尺寸
        if max_square_landscape > max_square_portrait:
            optimal_size_mm = int(max_square_landscape / mm * 2) / 2
            orientation = "横向"
        else:
            optimal_size_mm = int(max_square_portrait / mm * 2) / 2
            orientation = "纵向"
        
        return optimal_size_mm, orientation
    
    def generate_calibration_board_set(self, output_dir="./calibration_boards"):
        """
        批量生成常用的标定板
        
        Args:
            output_dir: 输出目录
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # 常用标定板配置
        board_configs = [
            # (rows, cols, square_size_mm, paper_size, description)
            (6, 9, 20, 'A4', '标准相机标定板'),
            (6, 9, 25, 'A4', '你的当前配置标定板'),
            (7, 10, 15, 'A4', '高精度相机标定板'),
            (8, 11, 12, 'A4', '小尺寸高精度标定板'),
            (6, 9, 30, 'A3', '大尺寸标定板'),
            (9, 12, 20, 'A3', 'A3大标定板'),
            (5, 8, 25, 'A4', '机器人手眼标定板'),
        ]
        
        generated_files = []
        
        print(f"开始批量生成标定板到目录: {output_dir}")
        print("=" * 70)
        
        for i, (rows, cols, square_size, paper_size, desc) in enumerate(board_configs, 1):
            try:
                filename = os.path.join(output_dir, 
                    f"{i:02d}_chessboard_{rows}x{cols}_{square_size}mm_{paper_size}.pdf")
                
                pdf_file = self.generate_chessboard_pdf(rows, cols, square_size, paper_size, filename)
                generated_files.append((pdf_file, desc))
                
                print(f"✓ 生成 {i}/7: {desc}")
                print(f"  参数: {rows}x{cols}, {square_size}mm, {paper_size}")
                
                # 同时生成高分辨率图像版本
                img_filename = filename.replace('.pdf', '.png')
                square_size_pixels = int(square_size * 12)  # 12 pixels per mm for high resolution
                img = self.generate_chessboard_image(rows, cols, square_size_pixels)
                cv2.imwrite(img_filename, img)
                print(f"  图像: {img_filename}")
                print()
                
            except Exception as e:
                print(f"✗ 生成失败 {i}/7: {desc} - {str(e)}")
        
        print("=" * 70)
        print(f"批量生成完成，共生成 {len(generated_files)} 个标定板")
        
        # 生成使用说明
        self.generate_usage_instructions(output_dir, generated_files)
        
        return generated_files
    
    def generate_usage_instructions(self, output_dir, generated_files):
        """生成使用说明文件"""
        instructions_file = os.path.join(output_dir, "使用说明.txt")
        
        with open(instructions_file, 'w', encoding='utf-8') as f:
            f.write("标定板使用说明\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("生成的标定板文件:\n")
            f.write("-" * 30 + "\n")
            for pdf_file, desc in generated_files:
                f.write(f"• {os.path.basename(pdf_file)}: {desc}\n")
            
            f.write("\n⭐ 重要：页面方向说明\n")
            f.write("-" * 30 + "\n")
            f.write("• 当棋盘格宽度超过纸张宽度时，会自动横向显示\n")
            f.write("• 文件名中包含'横向'或'纵向'标识\n")
            f.write("• 横向显示时，请将纸张横向放置打印\n\n")
            
            f.write("打印注意事项:\n")
            f.write("-" * 30 + "\n")
            f.write("1. 使用激光打印机或高质量喷墨打印机\n")
            f.write("2. 打印设置选择'实际尺寸'或'100%缩放'\n")
            f.write("3. 注意文件名中的方向提示（横向/纵向）\n") 
            f.write("4. 使用较厚的纸张(>120g/m²)避免弯曲\n")
            f.write("5. 确保打印后方格尺寸准确(可用尺子测量)\n")
            f.write("6. 将标定板平整地贴在刚性平板上\n\n")
            
            f.write("标定板选择建议:\n")
            f.write("-" * 30 + "\n")
            f.write("• 相机标定: 推荐 6x9 或 7x10 配置\n")
            f.write("• 手眼标定: 推荐 5x8 或 6x9 配置\n")
            f.write("• 高精度要求: 选择小方格尺寸(12-15mm)\n")
            f.write("• 远距离标定: 选择大方格尺寸(25-30mm)\n\n")
            
            f.write("ROS2标定参数设置:\n")
            f.write("-" * 30 + "\n")
            f.write("在你的标定程序中设置对应参数:\n")
            f.write("• chessboard_size: '列数,行数' (内角点数)\n")
            f.write("• square_size: 方格尺寸(米为单位)\n")
            f.write("例如: 6x9标定板，25mm方格 -> chessboard_size: '9,6', square_size: 0.025\n\n")
            
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        print(f"使用说明已保存: {instructions_file}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='标定板生成器 - 智能方向选择')
    parser.add_argument('--rows', type=int, default=6, help='棋盘格行数(内角点数，默认:6)')
    parser.add_argument('--cols', type=int, default=9, help='棋盘格列数(内角点数，默认:9)')
    parser.add_argument('--square-size', type=float, default=25, help='方格尺寸(mm，默认:25)')
    parser.add_argument('--paper-size', choices=['A4', 'A3', 'A2', 'Letter', 'Legal'], 
                       default='A4', help='纸张尺寸(默认:A4)')
    parser.add_argument('--output', type=str, help='输出文件名')
    parser.add_argument('--output-dir', type=str, default='./calibration_boards', 
                       help='输出目录(默认:./calibration_boards)')
    parser.add_argument('--batch', action='store_true', help='批量生成常用标定板')
    parser.add_argument('--image-only', action='store_true', help='只生成图像文件(PNG)')
    parser.add_argument('--auto-size', action='store_true', help='自动计算最佳方格尺寸')
    parser.add_argument('--check-size', action='store_true', help='检查指定配置的尺寸适配性')
    
    args = parser.parse_args()
    
    generator = CalibrationBoardGenerator()
    
    try:
        if args.check_size:
            # 检查尺寸适配性
            page_width, page_height, is_landscape, board_w, board_h = \
                generator.determine_page_orientation(args.rows, args.cols, args.square_size, args.paper_size)
            
            print(f"尺寸检查结果:")
            print(f"  棋盘格: {args.rows}x{args.cols} ({args.cols+1}x{args.rows+1}个方格)")
            print(f"  方格尺寸: {args.square_size}mm")
            print(f"  总尺寸: {board_w:.1f} x {board_h:.1f} mm")
            print(f"  纸张: {args.paper_size}")
            print(f"  推荐方向: {'横向' if is_landscape else '纵向'}")
            return 0
            
        elif args.batch:
            # 批量生成
            generator.generate_calibration_board_set(args.output_dir)
        else:
            # 单个生成
            if args.auto_size:
                optimal_size, orientation = generator.calculate_optimal_square_size(
                    args.rows, args.cols, args.paper_size)
                print(f"推荐方格尺寸: {optimal_size} mm ({orientation}显示)")
                square_size = optimal_size
            else:
                square_size = args.square_size
            
            if args.image_only:
                # 只生成图像
                square_size_pixels = int(square_size * 12)  # 12 pixels per mm
                img = generator.generate_chessboard_image(
                    args.rows, args.cols, square_size_pixels)
                
                if args.output:
                    filename = args.output
                else:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"chessboard_{args.rows}x{args.cols}_{square_size:.0f}mm_{timestamp}.png"
                
                os.makedirs(args.output_dir, exist_ok=True)
                filepath = os.path.join(args.output_dir, filename)
                cv2.imwrite(filepath, img)
                print(f"图像已生成: {filepath}")
            else:
                # 生成PDF
                os.makedirs(args.output_dir, exist_ok=True)
                if args.output:
                    filepath = os.path.join(args.output_dir, args.output)
                else:
                    filepath = None
                
                pdf_file = generator.generate_chessboard_pdf(
                    args.rows, args.cols, square_size, args.paper_size, filepath)
                
                print(f"\nROS2参数设置:")
                print(f"  chessboard_size: '{args.cols},{args.rows}'")
                print(f"  square_size: {square_size/1000:.3f}")
    
    except Exception as e:
        print(f"错误: {str(e)}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main()) 