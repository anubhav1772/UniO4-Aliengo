import numpy as np
import lcm
import time

import math
import select
import threading

'''Subscribe to camera_state_python (on which gravity and steps data are being published)
   inside lcm_position.cpp (under unitree_legged_sdk)
   For this to work, we need to generate python lcm message PositionGravityState
   
   Generate Python Bindings
   -------------------------------------------
   Use the `lcm-gen` tool to generate Python bindings for your LCM message types. 
   For example, if we have a message definition file named camera_lcm_msgs.lcm
   lcm-gen -p camera_lcm_msgs.lcm
   
   In the Terminal (before running this file)
   -------------------------------------------
   export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
   # modify enp3s0f1 with your own value (ip link show)
   sudo ifconfig enp3s0f1 multicast
   sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp3s0f1
'''

from go1_gym_deploy.lcm_types.PositionGravityState import PositionGravityState

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

def poll(cb=None):
    t = time.time()
    try:
        while True:
            timeout = 0.01
            rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
            if rfds:
                print("message received!")
                lc.handle()
                # print(f'Freq {1. / (time.time() - t)} Hz'); t = time.time()
            else:
                continue
                # print(f'waiting for message... Freq {1. / (time.time() - t)} Hz'); t = time.time()
                # if cb is not None:
                #   cb()
    except KeyboardInterrupt:
        pass

def _camera_cb(channel, data):
    msg = PositionGravityState.decode(data)
    
    gravity = np.array(msg.gravity)
    print(msg.gravity)

    camera_data = np.array(msg.data)
    print(camera_data)

def spin():
    run_thread = threading.Thread(target=poll, daemon=False)
    run_thread.start()


obs_camera_subscribtion = lc.subscribe("camera_state_python", _camera_cb)

poll(lc)
# spin()
