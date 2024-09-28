from pymavlink import mavutil
import time

master=mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

while True:

    msg=master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Heartbeat from CUBE: {msg}")
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        81,
        0, 0, 0, 0, 0, 0
    )

    time.sleep(1)
