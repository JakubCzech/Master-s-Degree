from __future__ import annotations

BO = 'little'


class DIO:
    def __init__(self, num_of_bits) -> None:
        self.num_of_bits = num_of_bits
        self._polarity = [1] * num_of_bits
        self._simulate = [0] * num_of_bits
        self._real = [0] * num_of_bits
        self._function = [0]

    @property
    def real(self):
        return self._list_to_int(self._real)

    @property
    def polarity(self):
        return self._list_to_int(self._polarity)

    @polarity.setter
    def polarity(self, values: tuple):
        din_num, state = values
        self._polarity[din_num] = 1 if state == 'NO' else 0

    @polarity.deleter
    def polarity(self):
        self._polarity = [1] * self.num_of_bits

    @property
    def simulate(self):
        return self._list_to_int(self._simulate)

    @simulate.setter
    def simulate(self, values: tuple):
        din_num, state = values
        self._simulate[din_num - 1] = state

    @simulate.deleter
    def simulate(self):
        self._simulate = [0] * self.num_of_bits

    @property
    def function(self):
        yield from self._function

    @function.setter
    def function(self, values: tuple = (0, 0)):
        din_num, fun = values
        self._function[din_num + 1] = fun

    def _list_to_int(self, list):
        _int = 0
        for i in range(self.num_of_bits):
            _int += list[i] << i
        return _int

    def parse_data(self, data):
        data = int.from_bytes(data[5:7], byteorder=BO, signed=True)
        data = [int(digit) for digit in bin(data)[bin(data).find('b') + 1:]]
        while len(data) < self.num_of_bits:
            data.insert(0, 0)
        return data

    def get_states(self, sim_data, real_data):
        sim_data = self.parse_data(sim_data)
        real_data = self.parse_data(real_data)
        _out = []
        for i in range(self.num_of_bits):
            if self._polarity[i]:
                _out.append(sim_data[i] | real_data[i])
            else:
                _out.append((not sim_data[i]) & (not real_data[i]))
        return _out
