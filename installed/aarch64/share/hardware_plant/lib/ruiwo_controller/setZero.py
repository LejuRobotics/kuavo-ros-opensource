#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import os
import pwd
import signal
import yaml
from ruiwo_actuator import RuiWoActuator

if __name__ == "__main__":
    try:
        joint_control = RuiWoActuator(setZero=True)
    except KeyboardInterrupt:
        pass 
    finally:
        joint_control.close()
        print('Exiting.')
 
