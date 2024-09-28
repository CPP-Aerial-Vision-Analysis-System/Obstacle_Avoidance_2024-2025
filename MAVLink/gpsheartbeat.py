 
from pymavlink import mavutil

def main():
    # - On Linux, it might be: "/dev/ttyACM0"

    connection_string = "/dev/ttyACM0"
    baud_rate = 57600  

    # Create a connection to the MAVLink device
    mav = mavutil.mavlink_connection(connection_string, baud=baud_rate)

    # Wait for a heartbeat before sending commands
    mav.wait_heartbeat()
    print("Heartbeat from system (system_id: %d component_id: %d)" % (mav.target_system, mav.target_component))
    
    # Request data to be sent 
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

    # Fetch messages in a loop
    try:
        while True:
            # Retrieve the next MAVLink message
            msg = mav.recv_match(type=['HEARTBEAT', 'AHRS2', 'GPS_RAW_INT'], blocking=True)
            if msg:
                if msg.get_type() == 'GPS_RAW_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    print(f"GPS_RAW_INT: lat = {lat:.7f}, lon = {lon:.7f}, alt = {alt:.2f} meters")
                elif msg.get_type() == 'AHRS2':
                    # Print out roll, pitch, yaw from AHRS2
                    roll = msg.roll
                    pitch = msg.pitch
                    yaw = msg.yaw
                    print(f"AHRS2: roll = {roll:.4f}, pitch = {pitch:.4f}, yaw = {yaw:.4f}")
                elif msg.get_type() == 'HEARTBEAT':
                    print("Heartbeat received: type=%s autopilot=%s base_mode=%s" % (msg.type, msg.autopilot, msg.base_mode))
                    print("Custom mode: %s" % msg.custom_mode)
    except KeyboardInterrupt:
        print("Exiting the program...")

if __name__ == '__main__':
    main()