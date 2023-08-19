from __future__ import annotations

from enum import Enum


class Controlword(Enum):
    num_of_bytes = 0x2B
    Power_on = 0x0F
    Power_off = 0x06
    Quick_stop = 0x0B
    Enable = 0x2F
    Start_abs_pos_1 = 0x3F
    Start_rel_pos_0 = 0x4F
    Start_rel_pos_1 = 0x5F
    Home_pos = 0x1F  # TODO: Check 0x0F 0x1F
    Clear = 0x80
    Zero = 0x00
    Test = 0x103F
    Start_abs_pos = 0x103F


class OperationMode(Enum):
    num_of_bytes = 0x2F
    Pos_control = 1
    Vel_control = 3
    Torque_control = 4
    Vel_control_quick = -3
    Pulse_train_control = -4
    Home_mode = 6
    Diff_compl = 7
