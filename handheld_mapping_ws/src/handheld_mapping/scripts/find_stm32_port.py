#!/usr/bin/env python3
"""Find STM32 Virtual COM Port via sysfs USB vendor/product ID."""

import glob
import os
import sys

STM32_VID = '0483'
STM32_PID = '5740'


def read_sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, FileNotFoundError):
        return None


def find_stm32():
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        tty_name = os.path.basename(dev)
        sysfs_dir = os.path.realpath(f'/sys/class/tty/{tty_name}')
        for _ in range(6):
            sysfs_dir = os.path.dirname(sysfs_dir)
            vid = read_sysfs(os.path.join(sysfs_dir, 'idVendor'))
            pid = read_sysfs(os.path.join(sysfs_dir, 'idProduct'))
            if vid == STM32_VID and pid == STM32_PID:
                return dev
    return None


if __name__ == '__main__':
    port = find_stm32()
    if port:
        print(port)
        sys.exit(0)
    ports = sorted(glob.glob('/dev/ttyACM*'))
    if ports:
        print(ports[0])
        sys.exit(0)
    print('/dev/ttyACM0')
    sys.exit(1)
