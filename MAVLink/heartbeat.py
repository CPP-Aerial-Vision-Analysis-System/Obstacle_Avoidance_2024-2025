from pymavlink import mavutil
import time

master=mavutil.mavlink_connection('udpin:localhost:14550')

while True:

    msg=master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Heartbeat from CUBE: {msg}")


    mav_msg = mavutil.mavlink.MAVLink_obstacle_distance_3d_message(
            time_boot_ms = int(100),  # Current time in microseconds
            sensor_type = int(0),
            obstacle_id=int(1),
            x=float(1),
            y=float(1),
            z=float(1),
            frame=int(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
            min_distance=float(0.5),
            max_distance=float(20.0)
        )
        
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mav_msg,
        0,
        81,
        0, 0, 0, 0, 0, 0
    )

    '''
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        81,
        0, 0, 0, 0, 0, 0
    )
    '''
    time.sleep(1)
