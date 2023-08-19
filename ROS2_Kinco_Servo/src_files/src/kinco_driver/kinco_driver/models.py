from __future__ import annotations


class Field:
    def __init__(
        self,
        address: bytearray,
        size: int,
        default_value: int,
        name: str = '',
    ) -> None:
        self.address = address
        self.size = size
        self.default_value = default_value
        self.name = name

    @property
    def cmd(self):
        _data = bytearray(9)
        _data[0] = self.size
        _data[1:4] = self.address
        _data[4:8] = self.default_value.to_bytes(
            4,
            byteorder='little',
            signed=True,
        )
        return _data
