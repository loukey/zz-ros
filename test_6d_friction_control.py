#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
6维数组和摩擦力控制逻辑测试
"""

import random
from src.gui.gui.config.global_config import GlobalVars

def test_6d_friction_control():
    """测试6维数组和摩擦力控制功能"""
    print("=== 6维数组和摩擦力控制测试 ===")
    
    # 清空数组
    GlobalVars.clear_array()
    print(f"初始状态:")
    print(f"  数组长度: {GlobalVars.get_array_length()}")
    print(f"  状态数组: {GlobalVars.get_state_array()}")
    print(f"  工作数组: {GlobalVars.get_work_array()}")
    print(f"  摩擦力数组: {GlobalVars.get_friction_array()}")
    
    # 添加19个正常范围的6维数据
    print(f"\n添加19个正常范围的6维数据...")
    for i in range(19):
        # 生成基准值附近的随机数据
        base_values = [10, 20, 30, 40, 50, 60]
        six_dim_data = [base + random.uniform(-5, 5) for base in base_values]
        GlobalVars.add_to_array(six_dim_data)
        print(f"  添加第{i+1}个数据: {[f'{x:.2f}' for x in six_dim_data]}")
    
    print(f"\n19个数据后的状态:")
    print(f"  数组长度: {GlobalVars.get_array_length()}")
    print(f"  状态数组: {GlobalVars.get_state_array()}")
    print(f"  工作数组: {GlobalVars.get_work_array()}")
    
    # 计算当前平均值
    avg_values = GlobalVars.get_average_value()
    print(f"  当前平均值: {[f'{x:.2f}' for x in avg_values]}")
    
    # 添加第20个数据，触发大幅变化（> 20）
    print(f"\n添加第20个数据（触发大幅变化）...")
    # 让第0、2、4维度变化超过20
    trigger_data = [
        avg_values[0] + 25,  # 维度0: +25 (>20)
        avg_values[1] + 5,   # 维度1: +5 (正常)
        avg_values[2] + 30,  # 维度2: +30 (>20)
        avg_values[3] - 5,   # 维度3: -5 (正常)
        avg_values[4] + 25,  # 维度4: +25 (>20)
        avg_values[5] + 10   # 维度5: +10 (正常)
    ]
    GlobalVars.add_to_array(trigger_data)
    print(f"  触发数据: {[f'{x:.2f}' for x in trigger_data]}")
    
    print(f"\n触发后的状态:")
    print(f"  数组长度: {GlobalVars.get_array_length()}")
    print(f"  状态数组: {GlobalVars.get_state_array()}")
    print(f"  工作数组: {GlobalVars.get_work_array()}")
    
    # 显示每个维度的差值
    for dim in range(6):
        diff = GlobalVars.get_dimension_difference(dim)
        print(f"  维度{dim}差值: {diff:.2f}")
    
    # 继续添加数据，测试状态为1时的逻辑
    print(f"\n继续添加数据，测试恢复逻辑...")
    for i in range(5):
        # 生成接近平均值的数据，让差值小于5
        recovery_data = [avg + random.uniform(-2, 2) for avg in avg_values]
        GlobalVars.add_to_array(recovery_data)
        
        print(f"  添加恢复数据{i+1}: {[f'{x:.2f}' for x in recovery_data]}")
        print(f"    状态数组: {GlobalVars.get_state_array()}")
        print(f"    工作数组: {GlobalVars.get_work_array()}")
        
        # 显示差值变化
        differences = []
        for dim in range(6):
            diff = GlobalVars.get_dimension_difference(dim)
            differences.append(diff)
        print(f"    各维度差值: {[f'{x:.2f}' for x in differences]}")
        print()
    
    # 测试负向触发（< -20）
    print(f"=== 测试负向触发 ===")
    GlobalVars.clear_array()
    
    # 添加19个数据
    for i in range(19):
        base_values = [100, 200, 300, 400, 500, 600]
        six_dim_data = [base + random.uniform(-5, 5) for base in base_values]
        GlobalVars.add_to_array(six_dim_data)
    
    avg_values = GlobalVars.get_average_value()
    print(f"基准平均值: {[f'{x:.2f}' for x in avg_values]}")
    
    # 添加负向触发数据
    negative_trigger_data = [
        avg_values[0] - 25,  # 维度0: -25 (<-20)
        avg_values[1] - 5,   # 维度1: -5 (正常)
        avg_values[2] - 30,  # 维度2: -30 (<-20)
        avg_values[3] + 5,   # 维度3: +5 (正常)
        avg_values[4] - 25,  # 维度4: -25 (<-20)
        avg_values[5] - 10   # 维度5: -10 (正常)
    ]
    GlobalVars.add_to_array(negative_trigger_data)
    
    print(f"负向触发数据: {[f'{x:.2f}' for x in negative_trigger_data]}")
    print(f"触发后状态数组: {GlobalVars.get_state_array()}")
    print(f"触发后工作数组: {GlobalVars.get_work_array()}")
    
    # 显示各维度差值
    for dim in range(6):
        diff = GlobalVars.get_dimension_difference(dim)
        print(f"维度{dim}差值: {diff:.2f}")

def test_friction_array_modification():
    """测试摩擦力数组修改"""
    print(f"\n=== 测试摩擦力数组修改 ===")
    
    print(f"默认摩擦力数组: {GlobalVars.get_friction_array()}")
    
    # 修改摩擦力数组
    new_friction = [1, 2, 3, 4, 5, 6]
    GlobalVars.set_friction_array(new_friction)
    print(f"修改后摩擦力数组: {GlobalVars.get_friction_array()}")
    
    # 测试修改后的效果
    GlobalVars.clear_array()
    
    # 添加19个数据
    for i in range(19):
        six_dim_data = [10 + random.uniform(-2, 2) for _ in range(6)]
        GlobalVars.add_to_array(six_dim_data)
    
    # 添加触发数据
    avg_values = GlobalVars.get_average_value()
    trigger_data = [avg + 25 for avg in avg_values]  # 所有维度都触发
    GlobalVars.add_to_array(trigger_data)
    
    print(f"使用新摩擦力数组触发后:")
    print(f"  工作数组: {GlobalVars.get_work_array()}")
    print(f"  状态数组: {GlobalVars.get_state_array()}")

def test_edge_cases():
    """测试边界情况"""
    print(f"\n=== 测试边界情况 ===")
    
    try:
        # 测试错误输入
        GlobalVars.add_to_array([1, 2, 3])  # 只有3维，应该报错
    except ValueError as e:
        print(f"正确捕获错误: {e}")
    
    try:
        # 测试错误的摩擦力数组设置
        GlobalVars.set_friction_array([1, 2, 3, 4])  # 只有4维，应该报错
    except ValueError as e:
        print(f"正确捕获错误: {e}")
    
    # 测试空数组情况
    GlobalVars.clear_array()
    print(f"空数组平均值: {GlobalVars.get_average_value()}")
    print(f"空数组最新值: {GlobalVars.get_latest_value()}")
    print(f"空数组维度差值: {GlobalVars.get_dimension_difference(0)}")

if __name__ == "__main__":
    test_6d_friction_control()
    test_friction_array_modification()
    test_edge_cases() 