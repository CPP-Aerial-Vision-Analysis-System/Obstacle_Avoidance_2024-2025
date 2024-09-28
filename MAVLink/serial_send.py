import time
from pymavlink import mavutil

pixhawk = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

def send_custom_data_to_arduino(data):
    target_system=1
    target_component=1


    pixhawk.mav.raw_imu_send(
        int(time.time()* 1000), data, data, data
        0, 0, 0
        0, 0, 0,
    )
    
def main():
    data=123  #some sample data
    while True:

        send_custom_data_to_arduino(data)
        time.sleep(1)

    if__name__== "__main__":
    main()

#with serial.Serial('/dev/ttyACM0', 9600, timeout=10) as ser:
    #while True:
        # led_on = input('Do you want the LED on? ')[0]
        #if led_on in 'pP':
            #ser.write(bytes('P\n','utf-8'))
            # IMAGE_START_CAPTURE
            
