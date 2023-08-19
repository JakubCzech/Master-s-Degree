from __future__ import annotations

import logging


ENCODER_RESOLUTION = 10000


def get_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
    # formatter = logging.Formatter
    # ('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)
    return logger


def calc_speed(speed):
    return int(speed * 512 * ENCODER_RESOLUTION / 1875)


def calc_acc(acc):
    return int(acc * 65536 * ENCODER_RESOLUTION / 4000000)
