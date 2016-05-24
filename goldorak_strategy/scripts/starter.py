#!/usr/bin/env python

STARTER_GPIO_PATH = "/sys/class/gpio/gpio34/value"

def poll():
    """
    Returns False when starter removed and True when starter is detected
    """
    try:
        f = open(STARTER_GPIO_PATH, 'r')
        return bool(int(f.read(1)))
    except IOError:
        return False
