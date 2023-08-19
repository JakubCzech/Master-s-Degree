from __future__ import annotations

from kinco_driver.models import Field
from kinco_driver.structs import Controlword
from kinco_driver.utils import get_logger


class RS232Telegram:
    BO = 'little'
    logger = get_logger('RS232Telegram')

    def __init__(self, id=None, data=None):
        if id:
            self._id = id
            self._data = bytearray(10)
            self._data[0] = id
        elif data:
            self._data = data
            self._print()
        else:
            raise ValueError('id or data must be provided')

    def parse_data(self, data):
        if data[1] not in [0x43, 0x4B, 0x4F]:
            self.logger.error('Not Readed data')
            raise ValueError('Data can not be readed')
        if data[1] == 0x43:
            return int.from_bytes(data[5:9], byteorder=self.BO, signed=True)
        elif data[1] == 0x4B:
            return int.from_bytes(data[5:7], byteorder=self.BO, signed=True)
        elif data[1] == 0x4F:
            return int.from_bytes(data[5:7], byteorder=self.BO, signed=True)

    def test_write(self, data):
        if data[1] == 0x60:
            return True
        elif data[1] == 0x80:
            self.logger.error('Not Writed data')
            return False

    def write(
        self,
        field: Field,
    ):
        try:
            self._data[1:9] = field.cmd
        except ValueError:
            raise ValueError('Field is not writable')
        else:
            self._data[9] = self.calc_checksum(self._data)
            return self._data

    def read(self, register=Controlword.Clear):
        self._data[1] = 0x40
        if not isinstance(register, bytearray):
            register = register.value
        self._data[2:5] = register
        self._data[5:9] = 0x00.to_bytes(4, byteorder=self.BO, signed=True)
        self._data[9] = self.calc_checksum(self._data)
        return self._data

    @staticmethod
    def calc_checksum(telegram):
        checksum = -sum(telegram[:9]) % 256
        return checksum

    def _print(self, text=None):
        text = '' if text is None else text
        try:
            self.logger.debug(
                f'{text}  ID:{self._data[0]}  CMD: {hex(self._data[1])}  '
                f'Index:{hex(self._data[2])} {hex(self._data[3])},  '
                f'Subindex:{self._data[4]},  self._data:'
                f'{self._data[5:9].hex()}  checksum: {hex(self._data[9])}',
            )
        except IndexError:
            self.logger.error('Not Readed data')
