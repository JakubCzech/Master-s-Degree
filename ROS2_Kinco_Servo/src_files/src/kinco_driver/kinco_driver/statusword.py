from __future__ import annotations


class Statusword:
    def __init__(self, data):
        value = int.from_bytes(data[5:7], byteorder='little', signed=True)
        value = [int(digit) for digit in bin(value)[bin(value).find('b') + 1:]]
        while len(value) < 16:
            value.insert(0, 0)
        self.ready_on = value[15]
        self.switch_on = value[14]
        self.operation_enabled = value[13]
        self.fault = value[12]
        self.voltage_enabled = value[11]
        self.quick_stop = value[10]
        self.switch_on_disabled = value[9]
        self.warning = value[7]
        self.remote = value[6]
        self.target_reached = value[5]
        self.internal_limit_active = value[4]
        self.setpoint_acknowledge = value[3]
        self.following_error = value[2]
        self.communication_found = value[1]
        self.reference_switch = value[0]
        self.print()

    def print(self):
        print(
            f'      Ready on {self.ready_on}  Switch on {self.switch_on} '
            f'Operation enabled {self.operation_enabled}  Fault {self.fault} '
            f'Voltage enabled {self.voltage_enabled}  Quick stop '
            f'{self.quick_stop} Switch on disabled {self.switch_on_disabled}'
            f'  Warning {self.warning} Remote {self.remote}  Target reached '
            f'{self.target_reached} Internal limit active '
            f'{self.internal_limit_active}  Setpoint acknowledge '
            f'{self.setpoint_acknowledge}  Following error '
            f'{self.following_error}  Communication found '
            f'{self.communication_found}  Reference switch '
            f'{self.reference_switch}',
        )
