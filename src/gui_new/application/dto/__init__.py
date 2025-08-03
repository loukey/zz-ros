"""
DTOs - Data Transfer Objects - 数据传输对象

用于在不同层之间传输数据，
避免直接暴露领域对象。
""" 

from .message_out_constructor import MessageOutConstructor

__all__ = ['MessageOutConstructor']
