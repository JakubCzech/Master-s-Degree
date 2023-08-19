from __future__ import annotations

from enum import Enum

from kinco_driver.models import Field
from kinco_driver.structs import Controlword
from kinco_driver.utils import calc_acc
from kinco_driver.utils import calc_speed


ONE_BYTE = 0x2F
TWO_BYTE = 0x2B
FOUR_BYTE = 0x23


class ServoDB:
    CONTROLWORD = Field(
        bytearray([0x40, 0x60, 0x00]),
        TWO_BYTE,
        Controlword.Enable.value,
    )
    INVERT_DIRECTION = Field(bytearray([0x7E, 0x60, 0x00]), ONE_BYTE, 1)

    TARGET_POSITION = Field(bytearray([0x7A, 0x60, 0x00]), FOUR_BYTE, 0)
    TARGET_SPEED = Field(
        bytearray([0xFF, 0x60, 0x00]),
        FOUR_BYTE,
        calc_speed(1000),
    )
    TARGET_TORQUE = Field(bytearray([0x71, 0x60, 0x00]), TWO_BYTE, 0)

    PROFILE_SPEED = Field(
        bytearray([0x81, 0x60, 0x00]),
        FOUR_BYTE,
        calc_speed(1500),
    )
    MAX_SPEED = Field(
        bytearray([0x80, 0x60, 0x00]),
        TWO_BYTE,
        calc_speed(250),
    )
    PROFILE_ACC = Field(
        bytearray([0x83, 0x60, 0x00]),
        FOUR_BYTE,
        calc_acc(200),
    )
    PROFILE_DEC = Field(
        bytearray([0x84, 0x60, 0x00]),
        FOUR_BYTE,
        calc_acc(100),
    )
# -17 platforma pozycjonuje się u góry przy INVERTDIRECTION=1
# -18 platforma pozycjonuje się u dołu przy INVERTDIRECTION=1
    HOMING_METHOD = Field(bytearray([0x98, 0x60, 0x00]), ONE_BYTE, -17)
    HOMING_SPEED_SWITCH = Field(
        bytearray([0x99, 0x60, 0x01]),
        FOUR_BYTE,
        calc_speed(500),
    )
    HOMING_CURRENT = Field(bytearray([0x99,0x60,0x04]),TWO_BYTE, 300)
    HOMING_SPEED_ZERO = Field(
        bytearray([0x99, 0x60, 0x02]),
        FOUR_BYTE,
        calc_speed(500),
    )
    HOMING_ACC = Field(bytearray([0x9A, 0x60, 0x00]), FOUR_BYTE, calc_acc(100))
    HOMING_POWER_ON = Field(bytearray([0x99, 0x60, 0x03]), ONE_BYTE, 1)

    DIN_MODE_0 = Field(bytearray([0x20, 0x20, 0x0D]), ONE_BYTE, 1)
    DIN_MODE_1 = Field(bytearray([0x20, 0x20, 0x0E]), ONE_BYTE, 6)
    DIN_SIMULATE = Field(bytearray([0x10, 0x20, 0x02]), TWO_BYTE, 0)

    DIN_POLARITY = Field(bytearray([0x10, 0x20, 0x01]), TWO_BYTE, 0xF7)
    DIN_FUNCTION_0 = Field(bytearray([0x10, 0x20, 0x03]), TWO_BYTE, 1)
    DIN_FUNCTION_1 = Field(bytearray([0x10, 0x20, 0x04]), TWO_BYTE, 2)
    DIN_FUNCTION_2 = Field(bytearray([0x10, 0x20, 0x05]), TWO_BYTE, 4)
    DIN_FUNCTION_3 = Field(bytearray([0x10, 0x20, 0x06]), TWO_BYTE, 0x2000)
    DOUT_SIMULATE = Field(bytearray([0x10, 0x20, 0x0E]), TWO_BYTE, 0)
    DOUT_POLARITY = Field(bytearray([0x10, 0x20, 0x0D]), TWO_BYTE, 0xFF)
    DOUT_FUNCTION_1 = Field(
        bytearray([0x10, 0x20, 0x0F]), TWO_BYTE, 1,
    )  # Ready
    DOUT_FUNCTION_2 = Field(
        bytearray([0x10, 0x20, 0x10]), TWO_BYTE, 2,
    )  # Error
    DOUT_FUNCTION_3 = Field(
        bytearray([0x10, 0x20, 0x11]), TWO_BYTE, 4,
    )  # Pos reached
    DOUT_FUNCTION_4 = Field(
        bytearray([0x10, 0x20, 0x12]), TWO_BYTE, 0x0400,
    )  # Home found

    @classmethod
    def get(cls):
        # yield each field
        for name, value in vars(cls).items():
            if isinstance(value, Field):
                value.name = name
                yield value


class RO_FIELDS(Enum):
    Statusword = bytearray([0x41, 0x60, 0x00])
    PosActual = bytearray([0x63, 0x60, 0x00])
    RealCurrent = bytearray([0x78, 0x60, 0x00])
    StatusInput = bytearray([0xFD, 0x60, 0x00])
    RealSpeed = bytearray([0x6C, 0x60, 0x00])
    DinReal = bytearray([0x10, 0x20, 0x0A])
    DoutReal = bytearray([0x10, 0x20, 0x14])
    # GroupCurrentLoop = bytearray([0xF6, 0x08, 0x10])
    # MaxCurrentCommand = bytearray([0x73, 0x60, 0x00])

    # Din_Pos0 = bytearray([0x20, 0x20, 0x01])
    # Din_Pos1 = bytearray([0x20, 0x20, 0x02])
    # Din_Pos2 = bytearray([0x20, 0x20, 0x03])
    # Din_Pos3 = bytearray([0x20, 0x20, 0x04])
    # Din_Pos4 = bytearray([0x20, 0x20, 0x10])
    # Din_Pos5 = bytearray([0x20, 0x20, 0x11])
    # Din_Pos6 = bytearray([0x20, 0x20, 0x12])
    # Din_Pos7 = bytearray([0x20, 0x20, 0x13])
    # Din_Speed0 = bytearray([0x20, 0x20, 0x05])
    # Din_Speed1 = bytearray([0x20, 0x20, 0x06])
    # Din_Speed2 = bytearray([0x20, 0x20, 0x07])
    # Din_Speed3 = bytearray([0x20, 0x20, 0x08])
    # Din_Speed4 = bytearray([0x20, 0x20, 0x14])
    # Din_Speed5 = bytearray([0x20, 0x20, 0x15])
    # Din_Speed6 = bytearray([0x20, 0x20, 0x16])
    # Din_Speed7 = bytearray([0x20, 0x20, 0x17])

    # Max_Following_Error = bytearray([0x65, 0x60, 0x00])
    # Target_Pos_Window = bytearray([0x67, 0x60, 0x00])
    # Position_Window_time = bytearray([0x25, 0x08, 0x09, 0x16])
    # Target_Speed_Window = bytearray([0x08, 0x25, 0x09])
    # Zero_Speed_Window = bytearray([0x10, 0x20, 0x18])
    # Zero_Speed_Time = bytearray([0xF9, 0x60, 0x14])
    # Soft_Positive_Limit = bytearray([0x60, 0x7D, 0x01])
    # Soft_Negative_Limit = bytearray([0x60, 0x7D, 0x02])
    # Limit_function = bytearray([0x10, 0x20, 0x19])

    # Homing_Offset = bytearray([0x7C, 0x60, 0x00])
    # Homing_Offset_Mode = bytearray([0x99, 0x60, 0x05])
    # Homing_Power_On = bytearray([0x99, 0x60, 0x03])
    # Kvp = bytearray([0xF9, 0x60, 0x01])
    # Kvi = bytearray([0xF9, 0x60, 0x02])
    # Kvi_32 = bytearray([0xF9, 0x60, 0x07])
    # Speed_Fb_N = bytearray([0xF9, 0x60, 0x05])

    # Kpp = bytearray([0xFB, 0x60, 0x01])
    # K_Velocity_FF = bytearray([0xFB, 0x60, 0x02])
    # K_Acc_FF = bytearray([0xFB, 0x60, 0x03])
    # Pos_Filter_N = bytearray([0xFB, 0x60, 0x05])

    # Error_State = bytearray([0x01, 0x26, 0x00])
