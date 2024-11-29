#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import sys
import smbus

class I2CReader:
    """
    Class to handle I2C communication with a battery management system.
    """
    def __init__(self, bus_number=7, device_address=0x55):
        """
        Initialize I2C communication.
        
        Args:
            bus_number (int): I2C bus number (default: 7)
            device_address (int): Device address on I2C bus (default: 0x55)
        """
        self.bus_number = bus_number
        self.device_address = device_address
        try:
            self.bus = smbus.SMBus(bus_number)
            print(f"I2C bus {bus_number} initialized successfully")
        except Exception as e:
            print(f"I2C initialization failed: {str(e)}")
            raise

    def read_remaining_capacity(self):
        """Read the remaining battery capacity."""
        try:
            data = self.bus.read_i2c_block_data(self.device_address, 0x04, 2)
            return (data[1] << 8) | data[0]
        except Exception as e:
            print(f"Failed to read battery capacity: {str(e)}")
            return None

    def read_battery_voltage(self):
        """
        Read the battery voltage.
        
        Returns:
            float: Battery voltage in volts
        """
        try:
            data = self.bus.read_i2c_block_data(self.device_address, 0x08, 2)
            raw_value = (data[1] << 8) | data[0]
            return raw_value * 0.001  # Convert to volts
        except Exception as e:
            print(f"Failed to read battery voltage: {str(e)}")
            return None

    def read_design_capacity(self):
        """Read the design capacity of the battery."""
        try:
            data = self.bus.read_i2c_block_data(self.device_address, 0x06, 2)
            return (data[1] << 8) | data[0]
        except Exception as e:
            print(f"Failed to read design capacity: {str(e)}")
            return None

    def __del__(self):
        """Clean up I2C connection on object destruction."""
        try:
            self.bus.close()
            print("I2C connection closed")
        except Exception as e:
            print(f"Failed to close I2C connection: {str(e)}")


class PangolinStatePublisher(Node):
    """
    ROS2 node for publishing battery state information.
    """
    def __init__(self):
        """Initialize the battery state publisher node."""
        super().__init__('Pangolin_State')
        self.battery_state = I2CReader()

        # Create publisher for battery state
        self.battery_state_publisher_ = self.create_publisher(
            BatteryState,
            '/battery_state',
            1  # QoS profile depth
        )

        # Create timer for periodic publishing
        timer_period = 0.01  # 100Hz
        self.battery_timer = self.create_timer(
            timer_period,
            self.battery_state_publisher
        )

    def battery_state_publisher(self):
        """Callback function to publish battery state information."""
        msg = BatteryState()
        
        # Set message header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery_frame"
        
        # Set battery state data
        msg.voltage = float(self.battery_state.read_battery_voltage())
        msg.capacity = float(self.battery_state.read_remaining_capacity())
        msg.design_capacity = float(self.battery_state.read_design_capacity())
        
        # Publish the message
        self.battery_state_publisher_.publish(msg)


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    state_publisher = PangolinStatePublisher()
    rclpy.spin(state_publisher)
    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()