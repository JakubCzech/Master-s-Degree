from __future__ import annotations

import inspect
import json
import sys

import pymodbus.exceptions as pymodbus_exceptions
import rclpy
from geometry_msgs.msg import Twist
from numpy import pi as PI
from pymodbus.client import ModbusTcpClient as ModbusClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import Endian
from rclpy.node import Node
from std_msgs.msg import String


def f_name():
    return inspect.stack()[1][3]


class AGV(Node):
    def __init__(self):
        """
        Create a new agv object
        """
        super().__init__('agv')
        # change log level to debug
        # rclpy.logging.set_logger_level(
        #     self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG
        # )
        # Odczytanie parametrów z pliku konfiguracyjnego
        self.load_parameters()
        # Inicjalizacja zmiennych
        self.init_variables()
        # Stworzenie klienta dla połączenia Modbus
        self.client = ModbusClient(
            host=self.mb_params['ip'],
            port=self.mb_params['port'],
            unit_id=self.mb_params['id'],
            auto_open=True,
            auto_close=True,
            timeout=0.1,
        )
        self.builder = BinaryPayloadBuilder(
            byteorder=Endian.Big, wordorder=Endian.Big,
        )
        # Inicjalizacja subskrybenta
        self.create_subscription(Twist, 'cmd_vel', self.update_next_twist, 10)
        # create publisher for dictionary with robot params as string
        self.publisher_ = self.create_publisher(String, 'diag_info', 10)
        # Stworzenie timera
        self.timer = self.create_timer(1.0 / self.frequency, self.update)
        # Stworzenie timera do odczytu parametrów robota
        self.diagnostic_timer = self.create_timer(
            1.0, self.read_params_from_robot,
        )
        # Włączenie trybu manualnego w robocie
        self.manual_on()
        self.get_logger().info(
            f'Frequency: {self.frequency} Hz, '
            f'Trans Vel Limit: {self.trans_vel_limit}',
        )

    def load_parameters(self):
        """
        Load parameters from the parameter server
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mb_base_data', 54500),
                ('mb_manual_ctrl', 65500),
                ('mb_tick_ms', 61756),
                ('mb_auto_ctrl', 33792),
                ('mb_manual', 33792),
                ('robot_controller_ip', '192.168.0.102'),
                ('robot_port', 502),
                ('robot_id', 1),
                ('trans_vel_limit', 200),
                ('rot_vel_limit', 100),
                ('delta_t', 500),
                ('frequency', 10),
            ],
        )
        self.mb_params = {
            'mb_base_data': self.get_parameter('mb_base_data').value,
            'mb_manual_ctrl': self.get_parameter('mb_manual_ctrl').value,
            'mb_tick_ms': self.get_parameter('mb_tick_ms').value,
            'mb_auto_ctrl': self.get_parameter('mb_auto_ctrl').value,
            'mb_manual': self.get_parameter('mb_manual').value,
            'ip': self.get_parameter('robot_controller_ip').value,
            'port': self.get_parameter('robot_port').value,
            'id': self.get_parameter('robot_id').value,
        }
        self.trans_vel_limit = int(self.get_parameter('trans_vel_limit').value)
        self.rot_vel_limit = int(self.get_parameter('rot_vel_limit').value)
        self.delta_t = int(self.get_parameter('delta_t').value)
        self.frequency = float(self.get_parameter('frequency').value)

        self.get_logger().debug('Loaded parameters')

    def init_variables(self):
        self.robot_time = 0
        self.twist_to_send = Twist()
        self.stop_order = False
        self.twist_received = False
        self.get_logger().debug('Variables initialized')

    def read_params_from_robot(self):
        try:
            data = self.client.read_holding_registers(
                self.mb_params['mb_base_data'], 14, 1,
            )
            decoder = BinaryPayloadDecoder.fromRegisters(
                data.registers, byteorder=Endian.Big,
            )
            decoded = {
                'FirmwareVer': decoder.decode_16bit_uint(),
                'RobotType': decoder.decode_8bit_uint(),
                'RobotStatus': decoder.decode_8bit_uint(),
                'ProgramStatus': decoder.decode_8bit_uint(),
                'ErrorCode': decoder.decode_8bit_uint(),
                'ScannerStatus': decoder.decode_8bit_uint(),
                'BatPercent': decoder.decode_8bit_uint(),
                'BatCurrent_mA': decoder.decode_16bit_int(),
                'EstimatedWorkTime_min': decoder.decode_16bit_uint(),
                'SpeedActual': decoder.decode_16bit_int(),
                'EngineLoadPercentLeft': decoder.decode_16bit_int(),
                'EngineLoadPercentRight': decoder.decode_16bit_int(),
                'PinOrLiftStatus': decoder.decode_8bit_uint(),
                'LmsCertainityPercent': decoder.decode_8bit_uint(),
                'LineXPos': decoder.decode_16bit_int(),
                'LastTagCode': decoder.decode_16bit_uint(),
                'SystickMs': decoder.decode_32bit_uint(),
            }
        except pymodbus_exceptions.ConnectionException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
        except pymodbus_exceptions.ModbusIOException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
        except AttributeError as e:
            self.get_logger().error(
                f'Error during {f_name()}  {e} {self.robot_time}',
            )
            return False
        else:
            data = json.dumps(decoded)
            msg = String(data=data)
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Robot params: {msg.data}')

    def __del__(self):
        self.stop()

    def update_next_twist(self, new_twist: Twist):
        """ """
        if (
            new_twist.angular.z == 0.0
            and new_twist.linear.x == 0.0
            and new_twist.linear.y == 0.0
        ):
            self.stop_order = True
            self.get_logger().debug('Stop twist')
        else:
            self.twist_to_send = new_twist

        self.twist_received = True

    def update_time(self):
        """
        Update the time of the robot

        """
        try:
            _time = self.client.read_holding_registers(
                self.mb_params['mb_tick_ms'], 2, 1,
            )
            decoder = BinaryPayloadDecoder.fromRegisters(
                _time.registers, byteorder=Endian.Big,
            )
            # TODO: check if this is correct
            decoded_time = decoder.decode_16bit_uint()
            decoded_time_2 = decoder.decode_16bit_uint()
            self.get_logger().debug(
                f'Time1: {decoded_time} : {decoded_time_2}',
            )
        except pymodbus_exceptions.ConnectionException as e:
            self.get_logger().error(
                f'Error during {f_name()}  {e} {self.robot_time}',
            )
            self.robot_time = 0
            return True
        except pymodbus_exceptions.ModbusIOException as e:
            self.get_logger().error(
                f'Error during {f_name()}  {e} {self.robot_time}',
            )
            self.robot_time = 0
            return True
        except AttributeError as e:
            self.get_logger().error(
                f'Error during {f_name()}  {e} {self.robot_time}',
            )
            self.robot_time = 0
            return True
        except Exception as e:
            self.get_logger().error(
                f'Error during {f_name()}  {e} {self.robot_time}',
            )
            self.robot_time = 0
            return True
        else:
            if decoded_time + self.delta_t >= 65536:
                self.robot_time = decoded_time - 65536 + self.delta_t
            else:
                self.robot_time = decoded_time + self.delta_t
            return True

    def update(self):
        """
        Update the robot register for sterring
        """
        # Sprawdzenie czy odebrano komendę
        if not self.twist_received:
            self.stop_order = True

        # Sprawdzenie czy konieczne jest zatrzymanie
        if self.stop_order:
            if self.stop():
                self.stop_order = False
            return
        # Pobranie aktualnego czasu robota
        if not self.update_time():
            return
        # Wysłanie danych do robota
        self.send_to_robot()
        self.twist_received = False

    def send_to_robot(self):
        """
        Stworzenie struktury Modbus,
        zawierającej dane do wysłania do robota
        """
        y = int(
            round(self.twist_to_send.linear.y * 1000.0),
        )  # konwersja z m/s na mm/s
        x = int(round(self.twist_to_send.linear.x * 1000.0))
        theta = int(
            round(-self.twist_to_send.angular.z * 180.0 / PI),
        )  # konwersja z rad/s na deg/s i zaokraglenie do calych stopni

        if x > 0.0:
            x = min(x, self.trans_vel_limit)
        elif x < 0.0:
            x = max(x, -self.trans_vel_limit)
        if y > 0.0:
            y = min(y, self.trans_vel_limit)
        elif y < 0.0:
            y = max(y, -self.trans_vel_limit)

        if theta > 0.0:
            theta = min(theta, self.rot_vel_limit)
        elif theta < 0.0:
            theta = max(theta, -self.rot_vel_limit)

        self.get_logger().debug(f'To send: X: {x}, Y: {y}, theta: {theta}')

        # Spakowanie danych do wysłania
        self.builder.reset()
        self.builder.add_16bit_uint(self.mb_params['mb_auto_ctrl'])
        self.builder.add_16bit_int(0)
        self.builder.add_16bit_int(int(x))
        self.builder.add_16bit_int(int(theta))
        self.builder.add_16bit_int(int(y))
        self.builder.add_32bit_uint(self.robot_time)
        payload = self.builder.to_registers()
        self.get_logger().debug(f'Payload: {payload}')

        try:
            self.client.write_registers(
                self.mb_params['mb_manual_ctrl'], payload,
            )
        except pymodbus_exceptions.ConnectionException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
            return False
        except pymodbus_exceptions.ModbusIOException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
            return False
        except AttributeError as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
            return False
        else:
            self.get_logger().debug('Send to robot')
            return True

    def manual_off(self):
        try:
            self.client.write_registers(
                self.mb_params['mb_manual_ctrl'], [0, 0, 0, 0, 0, 0, 0],
            )
        except pymodbus_exceptions.ConnectionException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
        except pymodbus_exceptions.ModbusIOException as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
        except AttributeError as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
            return False
        else:
            self.get_logger().debug('Manual off')

    def manual_on(self):
        try:
            self.client.write_registers(
                self.mb_params['mb_manual_ctrl'],
                [self.mb_params['mb_manual'], 0, 0, 0, 0, 0, 0],
            )
        except pymodbus_exceptions.ConnectionException as e:
            self.get_logger().error(f'Error during {__name__} {e}')
        except pymodbus_exceptions.ModbusIOException as e:
            self.get_logger().error(f'{e}')
        except AttributeError as e:
            self.get_logger().error(f'Error during {f_name()}  {e}')
            return False
        else:
            self.get_logger().debug('Manual on')

    def stop(self):
        if self.update_time():
            try:
                self.builder.reset()
                self.builder.add_16bit_uint(self.mb_params['mb_manual'])
                self.builder.add_16bit_int(0)
                self.builder.add_16bit_int(0)
                self.builder.add_16bit_int(0)
                self.builder.add_16bit_int(0)
                self.builder.add_32bit_uint(self.robot_time)
                payload = self.builder.to_registers()
                self.client.write_registers(
                    self.mb_params['mb_manual_ctrl'], payload,
                )
            except pymodbus_exceptions.ConnectionException as e:
                self.get_logger().error(f'Error during {__name__} {e}')
                return False
            except pymodbus_exceptions.ModbusIOException as e:
                self.get_logger().error(f'Error during {__name__} {e}')
                return False
            except AttributeError as e:
                self.get_logger().error(f'Error during {f_name()}  {e}')
                return False
            else:
                return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    robot = AGV()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()
