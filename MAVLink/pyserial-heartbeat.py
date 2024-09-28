from pymavlink import mavutil
# import serial

# Function to handle incoming MAVLink messages
def handle_mavlink_message(msg):
    if msg.get_type() == 'COMMAND_LONG' and msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
        # Execute code to trigger the camera to take an image
        print("Received IMAGE_START_CAPTURE command. Triggering camera...")
        # ser.write(bytes('P\n','utf-8'))

# Main function
def main():
    # Connect to the MAVLink stream (adjust port as necessary)
    master = mavutil.mavlink_connection('/dev/ttyACM0')

    # Wait for the IMAGE_START_CAPTURE command
    while True:
        msg = master.recv_match()
        if msg:
            handle_mavlink_message(msg)

if __name__ == "__main__":
    main()
