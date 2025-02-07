from pymavlink import mavutil
import time

master=mavutil.mavlink_connection('/dev/ttyTHS0',baud=115200)

while True:

    msg=master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Heartbeat from CUBE: {msg}")

    start_time = int(round(time.time() * 1000))
    current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
    current_time_ms = current_milli_time()

    master.mav.obstacle_distance_3d_send(
            
            current_time_ms,  # Current time in microseconds
            0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            65535,
            float(24),
            float(.01),
            float(5.01),
            float(0.0001),
            float(24)
        )
    
    text_msg = 'ROS2Mav: ' + "sent obstacle data"
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    time.sleep(.1)
