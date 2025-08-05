#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from uniconlink_flex_interfaces.msg import UniconCommand, UniconResponse, VariantValue
import time


class UnicronlinkFlexExample(Node):
    def __init__(self):
        super().__init__('uniconlink_flex_example')
        
        # Create publisher for commands
        self.unicon_tx = self.create_publisher(UniconCommand, 'unicon_tx', 10)
        
        # Create subscriber for responses
        self.unicon_rx = self.create_subscription(
            UniconResponse,
            'unicon_rx',
            self.response_callback,
            10
        )
        
        # Timer for sending example commands
        self.timer = self.create_timer(5.0, self.send_example_commands)
        
        self.get_logger().info('UniconlinkFlex example node started')
        
    def response_callback(self, msg):
        """Callback for receiving responses from the microcontroller"""
        self.get_logger().info(f'Received response with {len(msg.indices)} values:')
        for i, (index, value) in enumerate(zip(msg.indices, msg.values)):
            value_str = self.variant_to_string(value)
            self.get_logger().info(f'  Index {index}: {value_str}')
    
    def variant_to_string(self, variant_value):
        """Convert VariantValue to string representation"""
        if variant_value.type == VariantValue.TYPE_BOOL:
            return f'bool({variant_value.bool_value})'
        elif variant_value.type == VariantValue.TYPE_INT:
            return f'int({variant_value.int_value})'
        elif variant_value.type == VariantValue.TYPE_FLOAT:
            return f'float({variant_value.float_value})'
        elif variant_value.type == VariantValue.TYPE_STRING:
            return f'string("{variant_value.string_value}")'
        else:
            return 'unknown'
    
    def create_variant_value(self, value):
        """Create VariantValue from Python value"""
        variant = VariantValue()
        
        if isinstance(value, bool):
            variant.type = VariantValue.TYPE_BOOL
            variant.bool_value = value
        elif isinstance(value, int):
            variant.type = VariantValue.TYPE_INT
            variant.int_value = value
        elif isinstance(value, float):
            variant.type = VariantValue.TYPE_FLOAT
            variant.float_value = value
        elif isinstance(value, str):
            variant.type = VariantValue.TYPE_STRING
            variant.string_value = value
        else:
            variant.type = VariantValue.TYPE_NONE
            
        return variant
    
    def send_read_command(self, index):
        """Send a READ command for a single variable"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_READ
        cmd.indices = [index]
        cmd.values = []
        
        self.get_logger().info(f'Sending READ command for index {index}')
        self.unicon_tx.publish(cmd)
    
    def send_write_command(self, index, value):
        """Send a WRITE command for a single variable"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_WRITE
        cmd.indices = [index]
        cmd.values = [self.create_variant_value(value)]
        
        self.get_logger().info(f'Sending WRITE command for index {index} with value {value}')
        self.unicon_tx.publish(cmd)
    
    def send_sync_read_command(self, indices):
        """Send a SYNC_READ command for multiple variables"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_SYNC_READ
        cmd.indices = indices
        cmd.values = []
        
        self.get_logger().info(f'Sending SYNC_READ command for indices {indices}')
        self.unicon_tx.publish(cmd)
    
    def send_sync_write_command(self, indices, values):
        """Send a SYNC_WRITE command for multiple variables"""
        if len(indices) != len(values):
            self.get_logger().error('Indices and values must have the same length')
            return
        
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_SYNC_WRITE
        cmd.indices = indices
        cmd.values = [self.create_variant_value(v) for v in values]
        
        self.get_logger().info(f'Sending SYNC_WRITE command for indices {indices} with values {values}')
        self.unicon_tx.publish(cmd)
    
    def send_example_commands(self):
        """Send example commands to demonstrate the functionality"""
        
        # Example 1: Single READ
        self.send_read_command(0)  # Read motor goal speed
        
        # Example 2: Single WRITE
        self.send_write_command(0, 123.45)  # Write motor goal speed
        
        # Example 3: SYNC_READ for multiple variables
        self.send_sync_read_command([0, 1, 3])  # Read goal speed, status, and position
        
        # Example 4: SYNC_WRITE for multiple variables
        self.send_sync_write_command([0, 2, 4], [0.1, True, "hello"])  # Write goal speed, enable, and name


def main(args=None):
    rclpy.init(args=args)
    
    node = UnicronlinkFlexExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()