import array
import json
import os
import types
from ctypes import *
from enum import Enum

from rclpy.node import Node

from rogilink_flex_interfaces.msg import Frame

TYPES = {
    'int8': c_int8,
    'int16': c_int16,
    'int32': c_int32,
    'int64': c_int64,
    'uint8': c_uint8,
    'uint16': c_uint16,
    'uint32': c_uint32,
    'uint64': c_uint64,
    'float32': c_float,
    'float64': c_double,
    'bool': c_bool,
    'int': c_int,
    'uint': c_uint,
    'long': c_long,
    'ulong': c_ulong,
    'longlong': c_longlong,
    'ulonglong': c_ulonglong,
    'float': c_float,
    'double': c_double,
    'char': c_char,
    'uchar': c_uint8,
}

# CATCHROBO_2024_WS = os.environ['CATCHROBO_2024_WS']
# with open(f'{CATCHROBO_2024_WS}/settings/rogilinkflex.json', 'r') as f:
#     CONFIG = json.load(f)


class DataTypeEnum(Enum):
    TUPLE = 1
    LIST = 2
    PRIMITIVE = 3
    STR = 4

class DataConverter:
    def __init__(self, msg_type: str | type):
        '''c言語のデータ型を指定して、そのデータ型に対応するバイト列と元のデータ型の変換を行うクラス'''
        if isinstance(msg_type, str):
            self.msg_type = eval(msg_type, {}, TYPES)
        else:
            self.msg_type = msg_type

        if isinstance(self.msg_type, tuple):
            self.sub_converters = [DataConverter(t) for t in self.msg_type]
            self.data_type = DataTypeEnum.TUPLE
        elif isinstance(self.msg_type, types.GenericAlias) and self.msg_type.__origin__ == list:
            self.sub_converter = DataConverter(self.msg_type.__args__[0])
            self.data_type = DataTypeEnum.LIST
        elif self.msg_type in TYPES.values():
            self.data_type = DataTypeEnum.PRIMITIVE
        elif self.msg_type == str:
            self.data_type = DataTypeEnum.STR
        else:
            raise ValueError('Invalid message type')
            
    def serialize(self, value) -> list[int]:
        '''データをバイト列に変換する'''
        match self.data_type:
            case DataTypeEnum.TUPLE:
                '''タプルの場合、各要素をシリアライズして連結する'''
                data = []
                for arg, converter in zip(value, self.sub_converters):
                    data.extend(converter.serialize(arg))
                return data
            case DataTypeEnum.LIST:
                '''リストの場合、要素数を先頭に追加して、各要素をシリアライズして連結する'''
                data = [len(value)]
                for v in value:
                    data.extend(self.sub_converter.serialize(v))
                return data
            case DataTypeEnum.PRIMITIVE:
                '''プリミティブ型の場合、そのままバイト列に変換する'''
                converted = cast(pointer(self.msg_type(value)), POINTER(c_uint8 * sizeof(self.msg_type))).contents
                return [int(v) for v in converted]
            case DataTypeEnum.STR:
                '''文字列の場合、文字コードに変換して、最後にnull文字を追加する'''
                data = [ord(c) for c in value]
                data.append(0) # null terminator
                return data

    def deserialize_impl(self, data: list[int]):
        '''バイト列をデータに変換し、データとバイト数を返す'''
        match self.data_type:
            case DataTypeEnum.TUPLE:
                '''タプルの場合、各要素をデシリアライズしてタプルにまとめる'''
                value = []
                index = 0
                for converter in self.sub_converters:
                    converted, size = converter.deserialize_impl(data[index:])
                    value.append(converted)
                    index += size
                return tuple(value), index
            case DataTypeEnum.LIST:
                '''リストの場合、要素数を先頭から取得して、その数だけデシリアライズしてリストにまとめる'''    
                length = data[0]
                result = []
                index = 1
                for i in range(length):
                    converted, size = self.sub_converter.deserialize_impl(data[index:])
                    result.append(converted)
                    index += size
                return result, index
            case DataTypeEnum.PRIMITIVE:
                '''プリミティブ型の場合、そのままデシリアライズして値を取得する'''
                size = sizeof(self.msg_type)
                cut_data = data[:size]
                converted = cast((c_uint8 * size)(*cut_data), POINTER(self.msg_type)).contents
                return converted.value, size
            case DataTypeEnum.STR:
                '''文字列の場合、null文字まで取得して、文字列に変換する'''
                index = 0
                while data[index] != 0:
                    index += 1
                return ''.join([chr(c) for c in data[:index]]), index + 1
            
    def deserialize(self, data: list[int]):
        '''バイト列をデータに変換する'''
        try:
            result, size = self.deserialize_impl(data)
        except IndexError as e:
            return None
        
        if size != len(data):
            print('Warning: Unused data')
        return result

class Publisher:
    '''ROS2のPublisherをラップして、データをバイト列に変換して送信する'''
    def __init__(self, node: Node, topic: str | int, msg_type: str | type, qos_profile: int = 10, device_id: int = 0):
        self.node = node
        if isinstance(topic, int):
            self.publisher = self.node.create_publisher(Frame, f'rogilink_general_transmission', qos_profile)
            self.frame_id = topic
        else:
            self.publisher = self.node.create_publisher(Frame, f'rogilink_transmission/{topic}', qos_profile)
            # for topic_config in CONFIG['transmission_messages']:
            #     if topic_config['name'] == topic:
            #         self.frame_id = topic_config['id']
            #         break
            self.frame_id = 0
        self.device_id = device_id
        self.converter = DataConverter(msg_type)
        self.topic_name = topic

    def publish(self, *args):
        '''データをバイト列に変換して送信する'''
        if self.converter.data_type != DataTypeEnum.TUPLE:
            data = self.converter.serialize(args)
        else:
            data = self.converter.serialize(args[0])
        msg = Frame()
        msg.data = array.array('B', data)
        msg.frame_id = self.frame_id
        msg.device_id = self.device_id
        msg.topic = self.topic_name
        self.publisher.publish(msg)

class Subscriber:
    '''ROS2のSubscriberをラップして、受信したバイト列をデータに変換して返す'''
    def __init__(self, node: Node, topic: str | int, msg_type: str | type, callback_function: callable, qos_profile: int = 10, device_id: int = 0, expand_tuple: bool = False):
        self.node = node
        self.subscriber = self.node.create_subscription(Frame, f'rogilink_reception/{topic}', self.callback, qos_profile)
        self.converter = DataConverter(msg_type)
        self.callback_function = callback_function
        self.expand_tuple = expand_tuple
        self.device_id = device_id

    def callback(self, msg: Frame):
        '''バイト列をデータに変換してコールバック関数に渡す'''
        if msg.device_id != self.device_id:
            return
    
        value = self.converter.deserialize(msg.data)
        if value is None:
            return # ignore incomplete data
        
        if self.expand_tuple and isinstance(value, tuple):
            if len(value) != len(self.converter.msg_type):
                return
            self.callback_function(*value)
        else:
            self.callback_function(value)
        
if __name__ == '__main__':
    pass