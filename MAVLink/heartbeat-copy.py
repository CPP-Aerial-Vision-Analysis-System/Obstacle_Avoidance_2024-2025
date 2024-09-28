from pymavlink import mavutil
import time

master=mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

try:

    while True:

        msg=master.recv_match(type='HEARTBEAT', blocking=True)
        print(f"Heartbeat from CUBE: {msg}")
        
        master.mavlink.MAV_AUTOPILOT_INVALID,(
        0,
        0,
        0,
        )

        time.sleep(1)
        
except KeyboardInterrupt:
        print("Exiting the program...")
