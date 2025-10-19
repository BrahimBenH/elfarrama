import serial
import time

ser = serial.Serial('COM11', 9600, timeout=0.1)
time.sleep(2)              # allow Arduino to reset
ser.reset_input_buffer()   # drop stray bytes
ser.reset_output_buffer()
ser.write(b"START\n")      # send handshake once

def read_last_line(ser, idle_wait=0.02):
    """Read incoming lines and return the last non-empty line.
    Stops when no new data arrives for idle_wait seconds."""
    last = ""
    last_activity = time.time()
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                last = line
            last_activity = time.time()
        else:
            if time.time() - last_activity >= idle_wait:
                break
            time.sleep(0.005)
    return last

while True:
    if ser.in_waiting > 0:
        command = read_last_line(ser)
        print("Arduino said:", command)

        if command == "GET_NUMBER":
            ser.write(b"42\n")
        elif command == "GET_COLOR":
            ser.write(b"Red\n")
        elif command == "GET_SHAPE":
            ser.write(b"Circle\n")
    time.sleep(0.01)

