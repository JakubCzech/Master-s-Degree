from __future__ import annotations

import threading

import rclpy
from kinco_driver.servodriver import ServoDriver
from rclpy.node import Node
from std_msgs.msg import Int64


class KincoDriver(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', ''),
                ('baudrate', 115200),
                ('target_high_topic', 'target_high'),
                ('state_topic', 'state'),
                ('frequency', 2.0),
            ],
        )
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.servo_driver = ServoDriver(dev=port, baudrate=baudrate)
        except  Exception as e:
            self.get_logger().error(f'Error: {e} check port: {port}, and servo connection')
            self.get_logger().error('Exiting...')
            raise e
        
        self._publish_frequency = 1.0 / self.get_parameter('frequency').value
        self._target_high = self.create_subscription(
            Int64,
            self.get_parameter(
                'target_high_topic',
            )
            .get_parameter_value()
            .string_value,
            self._target_high_callback,
            10,
        )
        self._state_publisher = self.create_publisher(
            Int64,
            self.get_parameter(
                'state_topic',
            )
            .get_parameter_value()
            .string_value,
            10,
        )
        self._servo_lock = threading.Lock()
        self.get_logger().debug(f'Timer frequency: {self._publish_frequency}')
       
        self.servo_driver.clean_error()
        self.servo_driver.enable()
        self.servo_driver.clean_error()
        self.servo_driver.read_din_status()
        self.get_logger().info('Start homing')
        self.servo_driver.start_homing()
        self.get_logger().info('End homing')
        self.create_timer(0.1, self._timer_callback)

    def _target_high_callback(self, msg: Int64):
        with self._servo_lock:
            if self.servo_driver.is_moving_end:
                if msg.data != self.servo_driver.position:
                    if int(msg.data) > 180 :
                        self.get_logger().info(f'Target position to low: {msg.data}')
                    else:
                        self.get_logger().info(f'New target position: {msg.data}')
                        self.servo_driver.target_position = int(msg.data)
                        self.servo_driver.go_to_position()
                else:
                    self.get_logger().info(
                        f'Target position: {msg.data} ,'
                        f'actual position: {self.servo_driver.position}',
                    )
            else:
                self.get_logger().info('Servo is moving')

    def _timer_callback(self):
        with self._servo_lock:
            _position = self.servo_driver.position
        self.get_logger().info(f'Position from timer: {_position}')
        self._state_publisher.publish(Int64(data=_position))


def main(args=None):
    rclpy.init(args=args)
    try:
        kinco_driver = KincoDriver()
        rclpy.spin(kinco_driver)
    except KeyboardInterrupt:
        kinco_driver.destroy_node()
    except Exception as e:
        kinco_driver.get_logger().error(f'Error: {e}')
        kinco_driver.destroy_node()
    finally:
        rclpy.shutdown()
