from __future__ import annotations

import glob
import os
import threading
import time
from datetime import datetime

from kinco_driver.db import RO_FIELDS
from kinco_driver.db import ServoDB
from kinco_driver.digital_io import DIO
from kinco_driver.models import Field
from kinco_driver.rs232telegram import RS232Telegram
from kinco_driver.statusword import Statusword
from kinco_driver.structs import Controlword
from kinco_driver.utils import get_logger
from serial import Serial
from serial import SerialException

ENCODER_RESOLUTION = 10000


class ServoDriver:
    def __init__(
        self,
        dev: str,
        baudrate: int,
    ) -> None:
        self.logger = get_logger(self.__class__.__name__)
        try:
            self._get_connection(port=dev, baudrate=baudrate)
        except Exception as e:
            raise Exception(f'Connection error: {e}')
        self.telegram = RS232Telegram(id=1)
        self._last_write_time = time.time()
        self._last_write_cmd = None
        self._min_write_interval = 0.1
        self._serial_lock = threading.Lock()

        self.init_servo_params()
        self.din = DIO(8)
        self.dout = DIO(4)
        self._during_homing = threading.Event()
        self._end_of_homing = threading.Event()
        self._during_moving = threading.Event()
        self._end_of_moving = threading.Event()

    def _get_connection(self, port: str, baudrate: int) -> None:
        self.logger.info(f"Connecting to {glob.glob('/dev/ttyUSB*')}")
        if port != '':
            try:
                self.connect = Serial(
                    port,
                    baudrate,
                    timeout=0.08,
                    write_timeout=0.08,
                    exclusive=True,
                )
            except SerialException:
                self.logger.error(f'Connection error on port {port}')
            else:
                self.logger.info(f'Connected to {port}')
                return 
        if os.getenv('SERVO_PORT') !='':
            try:    
                self.connect = Serial(
                    os.getenv('SERVO_PORT'),
                    baudrate,
                    timeout=0.08,
                    write_timeout=0.08,
                    exclusive=True,
                )
            except SerialException:
                self.logger.error(f'Connection error on port {port}')
            else:
                self.logger.info(f'Connected to {port}')
                return
        if glob.glob('/dev/ttyUSB*'):
            for port in glob.glob('/dev/ttyUSB*'):
                self.logger.info(f"Trying to connect to {port}")
                try:
                    self.connect = Serial(
                        port,
                        baudrate,
                        timeout=0.08,
                        write_timeout=0.08,
                        exclusive=True,
                    )
                except SerialException:
                    self.logger.error(f'Connection error on port {port}')
                else:
                    self.logger.info(f'Connected to {port}')
                    return
          
        raise Exception('No servo connection found')

    def init_servo_params(self):
        for field in ServoDB.get():
            self._write(field)
        with self._serial_lock:
            self.connect.flush()
            self.connect.reset_input_buffer()
            self.connect.reset_output_buffer()

    def _test_write(self):
        _data = None
        _num_of_tries = 0
        with self._serial_lock:
            while not _data:
                try:
                    self.logger.debug('Testing write, waiting for response')
                    _data = self.connect.read(10)
                    self.connect.reset_input_buffer()
                    self.connect.reset_output_buffer()
                    self.logger.debug('Testing write,resposne received')
                except SerialException as e:
                    self.logger.error(e)
                    self._connection_reset()
                _num_of_tries += 1
                if not _data and _num_of_tries > 5:
                    self.connect.write(self._last_write_cmd)
        if self.telegram.test_write(_data):
            return True
        else:
            return False

    def _connection_reset(self):
        self.connect.flush()
        self.connect.reset_input_buffer()
        self.connect.reset_output_buffer()
        self.connect.send_break(1.0)
        self.connect.close()
        time.sleep(1.5)
        # remove self connect from serial
        self.connect = Serial(
            self.connect.port,
            self.connect.baudrate,
            timeout=0.08,
            write_timeout=0.08,
        )
        self.logger.info(f'Connection reset {datetime.now()}')

    def _write(self, field: Field):
        while time.time() - self._last_write_time < self._min_write_interval:
            time.sleep(0.01)
        _write_result = False
        _command = self.telegram.write(field)
        with self._serial_lock:
            while not _write_result:
                try:
                    _write_result = self.connect.write(_command)
                    self.connect.reset_output_buffer()
                except SerialException as e:
                    self.logger.error(f'Reading error {e}')
                    self._connection_reset()
        self._last_write_time = time.time()
        self._last_write_cmd = _command

        if self._test_write():
            self.logger.debug(f'{field.name} writed')
            return True
        else:
            self.logger.error(f'{field.name} not writed')
            return False

    def _read(self, address: RO_FIELDS):
        while time.time() - self._last_write_time < self._min_write_interval:
            time.sleep(0.01)
        _write_result = False
        _command = self.telegram.read(address)

        while not _write_result:
            with self._serial_lock:
                try:
                    self.logger.debug(f'Reading {address}')
                    _write_result = self.connect.write(_command)
                    self.connect.reset_output_buffer()
                except SerialException as e:
                    self.logger.error(f'Reading error {e}')
                    self._connection_reset()

        self._last_write_time = time.time()

        _data = None
        _num_of_tries = 0
        while not _data:
            with self._serial_lock:
                try:
                    _data = self.connect.read(10)
                    self.connect.reset_input_buffer()
                    self.connect.reset_output_buffer()
                except SerialException as e:
                    self.logger.error(f'Reading error {e}')
                    self._connection_reset()
                _num_of_tries += 1
                if not _data and _num_of_tries > 5:
                    self.connect.write(_command)

        self._last_write_cmd = _command
        return _data

    # Actual parameters
    @property
    def is_moving_end(self):
        if self._during_moving.is_set():
            self._end_of_moving.wait()
            return True
        else:
            return True

    @property
    def is_homing_end(self):
        if self._during_homing.is_set():
            self._end_of_homing.wait()
            return True
        else:
            return True

    @property
    def speed(self):
        return self.telegram.parse_data(self._read(RO_FIELDS.RealSpeed))

    @property
    def status(self):
        return Statusword(self._read(RO_FIELDS.Statusword))

    @property
    def position(self):
        _pos = self.telegram.parse_data(self._read(RO_FIELDS.PosActual))
        if _pos is not None:
            return round(_pos * 1.0 / ENCODER_RESOLUTION)
        else:
            self.logger.error('Position is not readed')
            with self._serial_lock:
                self.connect.flush()
                self.connect.reset_input_buffer()
                self.connect.reset_output_buffer()

    # Target parameters
    @property
    def target_speed(self):
        return self.telegram.parse_data(
            self._read(ServoDB.TARGET_SPEED.address),
        )

    @target_speed.setter
    def target_speed(self, speed):
        if self.op_mode != 'velocity':
            self.logger.warning(
                'Servo is not in velocity mode, setting to velocity mode',
            )
            self.set_velocity_mode()
        ServoDB.TARGET_SPEED.default_value = int(self.calc_speed(speed))
        self._write(ServoDB.TARGET_SPEED)

    @property
    def target_position(self):
        return self.telegram.parse_data(
            self._read(ServoDB.TARGET_POSITION.address),
        )

    @target_position.setter
    def target_position(self, position):
        ServoDB.TARGET_POSITION.default_value = int(
            position * ENCODER_RESOLUTION,
        )
        self._write(ServoDB.TARGET_POSITION)

    @property
    def target_acc(self):
        return self.telegram.parse_data(
            self._read(ServoDB.PROFILE_ACC.address),
        )

    @target_acc.setter
    def target_acc(self, acc):
        ServoDB.HOMING_ACC.default_value = acc
        self._write(ServoDB.HOMING_ACC)

    # API functions
    def clean_error(self):
        self.din.simulate = 2, 1
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        self._write(ServoDB.DIN_SIMULATE)
        self.din.simulate = 1, 0
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        self._write(ServoDB.DIN_SIMULATE)
        self.din.simulate = 1, 1
        self.din.simulate = 2, 0
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        self._write(ServoDB.DIN_SIMULATE)


    def start_homing(self):
        # ServoDB.DIN_POLARITY.default_value = self.din.polarity
        self._during_homing.set()
        self.din.polarity = 4, 'NO'
        self.din.polarity = 3, 'NO'
        ServoDB.DIN_POLARITY.default_value = self.din.polarity
        if self._write(ServoDB.DIN_POLARITY):
            self._during_homing.set()
            while not self.dout.get_states(
                self._read(ServoDB.DOUT_SIMULATE.address),
                self._read(RO_FIELDS.DoutReal),
            )[1]:
                pass
            self._end_of_homing.set()
            self._during_homing.clear()
            self.logger.info(f'Homing finished {datetime.now()}')

    def go_to_position(self):
        self.din.simulate = 5, 1
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        if self._write(ServoDB.DIN_SIMULATE):
            self._during_moving.set()
            while not self.dout.get_states(
                self._read(ServoDB.DOUT_SIMULATE.address),
                self._read(RO_FIELDS.DoutReal),
            )[2]:
                pass
            self.din.simulate = 5, 0
            ServoDB.DIN_SIMULATE.default_value = self.din.simulate
            self._write(ServoDB.DIN_SIMULATE)
            self._end_of_moving.set()
            self._during_moving.clear()

    def stop(self):
        ServoDB.CONTROLWORD.default_value = Controlword.Power_off.value
        self._write(ServoDB.CONTROLWORD)
        self.clear_din()

    def enable(self):
        self.logger.info('Enabling servo')
        ServoDB.CONTROLWORD.default_value = Controlword.Enable.value
        self._write(ServoDB.CONTROLWORD)
        self.din.simulate = 1, 1
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        self._write(ServoDB.DIN_SIMULATE)

    def read_din_status(self):
        sim = self._read(ServoDB.DIN_SIMULATE.address)
        real = self._read(RO_FIELDS.DinReal)
        _msg = 'Din status: '
        for i, value in enumerate(self.din.get_states(sim, real)):
            _msg += f'DIN{abs(i-7)+1}: {value}, '
        self.logger.info(_msg)

    def read_dout_status(self):
        sim = self._read(ServoDB.DOUT_SIMULATE.address)
        real = self._read(RO_FIELDS.DoutReal)
        _msg = 'Dout status: '
        for i, value in enumerate(self.dout.get_states(sim, real)):
            _msg += f'DOUT{abs(i-3)+1}: {value}, '
        self.logger.info(_msg)

    def clear_din(self):
        self.din = DIO(8)
        ServoDB.DIN_SIMULATE.default_value = self.din.simulate
        self._write(ServoDB.DIN_SIMULATE)

    def __del__(self):
        self.stop()
        self.connect.close()
