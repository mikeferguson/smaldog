#!/usr/bin/env python

from smaldog.eth_bridge import *
from smaldog.robot_defs import *
import time

eth = EthBridge()
defs = SMALdog()

for i in range(12):
    eth.setPosition(i+1, defs.neutrals[i])
    time.sleep(0.1)
