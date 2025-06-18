#!/usr/bin/env python3
"""
棋盘格标定板生成器
生成标准的相机标定用棋盘格图案
"""

import cv2
import numpy as np
import argparse
import os


def generate_chessboard(cols, rows, square_size_mm, dpi=300, border_mm=20):
    """
    生成棋盘格标定板
    
    Args:
        cols: 内角点列数
        rows: 内角点行数 
        square_size_mm: 方格尺寸(毫米)
        dpi: 图像分辨率
        border_mm: 边框宽度(毫米)
    
    Returns:
        numpy.ndarray: 棋盘格图像
    """
    
    # 计算像素尺寸
    mm_to_pixel = dpi / 25.4  # 1英寸 = 25.4毫米
    square_size_px = int(square_size_mm * mm_to_pixel)
    border_px = int(border_mm * mm_to_pixel)
    
    # 计算图像尺寸
    # 棋盘格实际方格数 = 内角点数 + 1
    board_cols = cols + 1
    board_rows = rows + 1
    
    # 图像总尺寸
    img_width = board_cols * square_size_px + 2 * border_px
    img_height = board_rows * square_size_px + 2 * border_px
    
    print(f"生成棋盘格参数:")
    print(f"  内角点数: {cols} × {rows}")
    print(f"  方格数: {board_cols} × {board_rows}")
    print(f"  方格尺寸: {square_size_mm}mm ({square_size_px}px)")
    print(f"  图像尺寸: {img_width} × {img_height} px")
    print(f"  物理尺寸: {img_width/mm_to_pixel:.1f} × {img_height/mm_to_pixel:.1f} mm")
    
    # 创建空白图像(白色背景)
    img = np.ones((img_height, img_width), dtype=np.uint8) * 255
    
    # 绘制棋盘格
    for row in range(board_rows):
        for col in range(board_cols):
            # 计算当前方格位置
            x1 = border_px + col * square_size_px
            y1 = border_px + row * square_size_px
            x2 = x1 + square_size_px
            y2 = y1 + square_size_px
            
            # 棋盘格模式：(row + col) % 2 决定黑白
            if (row + col) % 2 == 1:
                # 绘制黑色方格
                cv2.rectangle(img, (x1, y1), (x2, y2), 0, -1)
    
    return img


def add_info_text(img, cols, rows, square_size_mm, dpi):
    """在图像上添加标定板信息"""
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    # 信息文本
    info_text = [
        f"Camera Calibration Chessboard",
        f"Inner corners: {cols} x {rows}",
        f"Square size: {square_size_mm}mm",
        f"DPI: {dpi}",
        f"Print at 100% scale"
    ]
    
    # 文本参数
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    color = (0, 0, 255)  # 红色
    thickness = 2
    
    # 在图像底部添加信息
    text_height = 30
    y_offset = img.shape[0] - len(info_text) * text_height - 20
    
    for i, text in enumerate(info_text):
        y = y_offset + i * text_height
        cv2.putText(img_color, text, (20, y), font, font_scale, color, thickness)
    
    return img_color


def main():
    parser = argparse.ArgumentParser(description='生成相机标定用棋盘格')
    parser.add_argument('--cols', type=int, default=9, help='内角点列数 (默认: 9)')
    parser.add_argument('--rows', type=int, default=6, help='内角点行数 (默认: 6)')
    parser.add_argument('--square_size', type=float, default=25.0, help='方格尺寸mm (默认: 25.0)')
    parser.add_argument('--dpi', type=int, default=300, help='图像分辨率DPI (默认: 300)')
    parser.add_argument('--border', type=float, default=20.0, help='边框宽度mm (默认: 20.0)')
    parser.add_argument('--output', type=str, default='chessboard_calibration.png', help='输出文件名')
    parser.add_argument('--with_info', action='store_true', help='在图像上添加标定板信息')
    
    args = parser.parse_args()
    
    print("="*60)
    print("棋盘格标定板生成器")
    print("="*60)
    
    # 生成棋盘格
    chessboard = generate_chessboard(
        args.cols, 
        args.rows, 
        args.square_size, 
        args.dpi, 
        args.border
    )
    
    # 是否添加信息文本
    if args.with_info:
        chessboard = add_info_text(chessboard, args.cols, args.rows, args.square_size, args.dpi)
    
    # 保存图像
    output_path = args.output
    cv2.imwrite(output_path, chessboard)
    
    # 计算文件信息
    file_size = os.path.getsize(output_path) / 1024  # KB
    
    print("\n" + "="*60)
    print("生成完成!")
    print("="*60)
    print(f"输出文件: {output_path}")
    print(f"文件大小: {file_size:.1f} KB")
    print(f"建议打印设置:")
    print(f"  - 纸张: A4 或更大")
    print(f"  - 打印质量: 高质量/照片质量")
    print(f"  - 缩放: 100% (实际尺寸)")
    print(f"  - 纸张类型: 普通纸或照片纸")
    print("="*60)
    
    # 显示预览
    try:
        # 缩放显示
        display_img = cv2.resize(chessboard, None, fx=0.3, fy=0.3)
        cv2.imshow('Chessboard Preview (按任意键关闭)', display_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except:
        print("无法显示预览图像")
    
    print(f"\n标定板已保存为: {output_path}")
    print("请使用高质量打印机打印，确保100%实际尺寸!")


if __name__ == '__main__':
    main() 