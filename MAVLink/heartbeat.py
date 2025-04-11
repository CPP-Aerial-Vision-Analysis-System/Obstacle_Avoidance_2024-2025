from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import time


import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

master= mavutil.mavlink_connection('tcp:localhost:5762', dialect='ardupilotmega')

start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
current_time_ms = current_milli_time()

while True:

    msg=master.recv_match(type='HEARTBEAT', blocking=False)
    if(msg):
        print(f"Heartbeat from CUBE: {msg}")

#     master.mav.heartbeat_send(
#             mavutil.mavlink.MAV_TYPE_GCS,
#             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
#             0,0,0)


    mav_msg = master.mav.obstacle_distance_3d_send(
            current_time_ms * 1000,    # us Timestamp (UNIX time or time since system boot)
            0,                  
            mavutil.mavlink.MAV_FRAME_BODY_FRD,                  
            65535,              
            float(1),	    
            float(0),       
            float(0),	    
            float(.01),       
            float(25)
        )

    
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    #     0,
    #     81,
    #     0, 0, 0, 0, 0, 0
    # )
    
    time.sleep(.01)
